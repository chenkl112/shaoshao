"""Motion envelopes, actual footprint safety and retained-task lifecycle."""
from concurrent.futures import Future
import math
import threading
import time
from unittest.mock import patch
from types import SimpleNamespace as NS
import unittest
import numpy as np

from navigation_runtime import CollisionGrid, RecoveryTask, SmoothVelocity, map_odom_correction, correction_jump
from navigation_recovery import RecoveryBridgeMixin


class MotionTests(unittest.TestCase):
    def test_bridge_keeps_filter_twist_when_pose_delivery_is_jittery(self):
        from navcar_navigation import NavigationBridge
        panel=NS(lock=threading.RLock(),navigation_control=NS(frames={
            'center_offset':[.1,-.05,0.],'base_frame':'navcar_center'}))
        published=[]
        bridge=NS(panel=panel,node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_000))),
                  odom_pub=NS(publish=published.append))
        for x in (0.,.02,2.,2.):
            msg=NS(header=NS(stamp=NS(sec=10,nanosec=0),frame_id='navcar_odom'),
                   pose=NS(pose=NS(position=NS(x=x,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.)),
                           covariance=[0.]*36),
                   twist=NS(twist=NS(linear=NS(x=.2,y=.1,z=0.),angular=NS(x=0.,y=0.,z=.5)),covariance=[0.]*36))
            NavigationBridge.odom(bridge,msg)
        self.assertEqual(len(published),4)
        for msg in published:
            self.assertAlmostEqual(msg.twist.twist.linear.x,.225)
            self.assertAlmostEqual(msg.twist.twist.linear.y,.15)
            self.assertEqual(msg.child_frame_id,'navcar_center')

    def test_turn_reverse_settle_and_jitter_obey_vector_bounds(self):
        limiter = SmoothVelocity()
        rng = np.random.default_rng(9)
        trace = []
        for i in range(1300):
            if i < 200:
                target = (.5, 0.)
            elif i < 450:
                target = (0., -.5)
            elif i < 700:
                target = (-.5, 0.)
            elif i < 1000:
                target = rng.uniform(-.5, .5, 2)
            else:
                target = (0., 0.)
            trace.append(limiter.step(target, .02))
        velocity = np.asarray(trace)
        acceleration = np.diff(velocity, axis=0)/.02
        jerk = np.diff(acceleration, axis=0)/.02
        self.assertLessEqual(np.linalg.norm(velocity, axis=1).max(), .5+1e-8)
        self.assertLessEqual(np.linalg.norm(acceleration, axis=1).max(), .3+1e-8)
        self.assertLessEqual(np.linalg.norm(jerk, axis=1).max(), 1.+1e-7)
        self.assertLess(np.linalg.norm(velocity[-1]), 1e-7)

    def test_emergency_and_stale_loop_do_not_coast_or_resume_old_profile(self):
        limiter = SmoothVelocity()
        for _ in range(200):
            limiter.step((.5, 0.), .02)
        self.assertEqual(limiter.step((.5, 0.), .02, emergency=True), (0., 0.))
        self.assertEqual(limiter.step((.5, 0.), .3), (0., 0.))
        self.assertLess(math.hypot(*limiter.step((.5, 0.), .02)), .001)
        self.assertEqual(limiter.step((math.nan, 0.), .02), (0., 0.))

    def test_irregular_send_intervals_and_uncommitted_preview_keep_bounds(self):
        limiter=SmoothVelocity()
        first=limiter.preview((.5,0.),.02)
        limiter.preview((0.,.5),.1)
        self.assertEqual(limiter.v,(0.,0.))
        self.assertEqual(limiter.a,(0.,0.))
        limiter.commit(first)
        rng=np.random.default_rng(11);velocities=[limiter.v];intervals=[]
        for i in range(800):
            dt=float(rng.uniform(.01,.12));intervals.append(dt)
            velocities.append(limiter.step((.5,0.) if i<300 else (0.,-.5),dt))
        measured=np.diff(velocities,axis=0)/np.asarray(intervals)[:,None]
        jerk=np.diff(measured,axis=0)/((np.asarray(intervals)[1:]+np.asarray(intervals)[:-1])/2)[:,None]
        self.assertLessEqual(np.linalg.norm(measured,axis=1).max(),.3+1e-8)
        self.assertLessEqual(np.linalg.norm(jerk,axis=1).max(),1.+1e-8)

    def test_soft_inflation_is_allowed_but_full_hull_and_unknown_are_checked(self):
        cells = np.full((160, 160), 180, np.uint8)
        cells[:, 130:] = 254
        grid = CollisionGrid(cells, .025, (-2., -2., 0.), 10., 'odom')
        self.assertTrue(grid.safe_segment((0., 0.), (.3, 0.), .317658))
        self.assertFalse(grid.safe_segment((0., 0.), (1., 0.), .317658))
        self.assertFalse(grid.safe_velocity((0., 0., 0.), (.5, 0.), .317658, 10.6))
        cells[70:90, 82:90] = 255
        grid = CollisionGrid(cells, .025, (-2., -2., 0.), 10., 'odom')
        self.assertFalse(grid.safe_segment((0., 0.), (.3, 0.), .317658))

    def test_closed_rear_selects_lateral_escape_and_closed_hull_holds(self):
        cells = np.zeros((160, 160), np.uint8)
        cells[:, :64] = 254  # Rear obstacle is beyond the current hull.
        grid = CollisionGrid(cells, .025, (-2., -2., 0.), 10., 'odom')
        escape = grid.escape((0., 0., 0.), .317658, previous=(.2, 0.))
        self.assertIsNotNone(escape)
        self.assertTrue(grid.safe_segment((0., 0.), escape, .317658))
        self.assertGreater(escape[0], -.1)
        cells[80, 80] = 254
        blocked = CollisionGrid(cells, .025, (-2., -2., 0.), 10., 'odom')
        self.assertIsNone(blocked.escape((0., 0., 0.), .317658))

    def test_narrow_passage_slows_before_entry_and_keeps_physical_hull(self):
        # 65 cm known corridor with soft inflation throughout, no added padding.
        resolution=.01
        cells=np.full((180,400),180,np.uint8)
        cells[:57,240:]=254
        cells[122:,240:]=254
        grid=CollisionGrid(cells,resolution,(-2.4,-.9,0.),10.,'odom')
        radius=.297658
        self.assertTrue(grid.safe_segment((.1,0.),(.9,0.),radius))
        self.assertFalse(grid.safe_segment((.1,.05),(.9,.05),radius))
        self.assertLessEqual(grid.passage_speed_limit((-.4,0.,0.),(.4,0.),radius,.4),.09)
        self.assertAlmostEqual(grid.passage_speed_limit((-.9,0.,0.),(-.4,0.),radius,.4),.4)
        self.assertAlmostEqual(grid.passage_speed_limit((-.4,0.,math.pi/2),(0.,-.4),radius,.4),
                               grid.passage_speed_limit((-.4,0.,0.),(.4,0.),radius,.4))

    def test_exact_fallback_does_not_add_grid_padding_or_erase_walls(self):
        cells=np.zeros((160,160),np.uint8)
        cells[:68,:]=254;cells[92:,:]=255  # Exactly 60 cm known free.
        grid=CollisionGrid(cells,.025,(-2.,-2.,0.),10.,'odom')
        self.assertTrue(grid.safe_segment((0.,0.),(.4,0.),.297658))
        self.assertFalse(grid.safe_segment((0.,.004),(.4,.004),.297658))
        self.assertFalse(grid.safe_segment((0.,0.),(.4,0.),.301))
        cells=np.zeros((100,100),np.uint8);cells[50,50]=254
        grid=CollisionGrid(cells,.025,(-1.25,-1.25,0.),10.,'odom')
        self.assertTrue(grid.safe_segment((-.2,-.1),(.2,-.1),.09))
        self.assertFalse(grid.safe_segment((-.2,-.1),(.2,-.1),.1))
        self.assertFalse(grid.safe_segment((.125,-.1),(.125,.1),.1))
        self.assertTrue(grid.safe_segment((-.05,-.05),(-.05,-.05),.07))
        self.assertFalse(grid.safe_segment((-.05,-.05),(-.05,-.05),.071))
        self.assertFalse(grid.safe_segment((-.1,-.1),(.1,.1),.001))

    def test_boundary_acceleration_preserves_thick_walls_and_unknown_interiors(self):
        cells=np.zeros((120,120),np.uint8)
        cells[30:90,45:75]=255
        grid=CollisionGrid(cells,.025,(-1.5,-1.5,0.),10.,'odom')
        self.assertFalse(grid.exact_safe_segment((0.,0.),(0.,.1),.01),
                         'a center buried in unknown cells cannot escape the boundary check')
        self.assertFalse(grid.exact_safe_segment((-.7,0.),(.7,0.),.01))
        self.assertTrue(grid.exact_safe_segment((-.7,-1.),(.7,-1.),.2))

    def test_centering_includes_a_protrusion_beside_the_current_hull(self):
        cells=np.zeros((200,200),np.uint8)
        cells[:68,:]=254;cells[132:,:]=254
        # Future walls are centered lower; the body still passes a protrusion
        # on the south wall. Looking only 25 cm ahead misses this constraint.
        cells[:70,:110]=254
        grid=CollisionGrid(cells,.01,(-1.,-1.,0.),10.,'odom')
        velocity=grid.centered_velocity((0.,0.,0.),(.08,0.),.297658)
        self.assertGreater(velocity[1],0.)

    def test_centering_corrects_toward_opposing_wall_midpoint_only(self):
        cells=np.zeros((160,160),np.uint8)
        cells[:67,80:]=254;cells[93:,80:]=254
        grid=CollisionGrid(cells,.025,(-2.,-2.,0.),10.,'odom')
        centered=grid.centered_velocity((-.1,.02,0.),(.08,0.),.297658)
        self.assertLess(centered[1],0.)
        diagonal=grid.centered_velocity((-.1,.02,0.),(.08,.02),.297658)
        self.assertLess(diagonal[1],0.,'corridor axis must override a sampled direction toward the wall')
        rotated=grid.centered_velocity((-.1,.02,math.pi/2),(0.,-.08),.297658)
        self.assertAlmostEqual(rotated[0],centered[1])
        cells[93:,80:]=0
        grid=CollisionGrid(cells,.025,(-2.,-2.,0.),10.,'odom')
        self.assertEqual(grid.centered_velocity((-.1,.02,0.),(.08,0.),.297658),(.08,0.))

    def test_rotated_grid_preserves_wall_and_boundary_checks(self):
        cells = np.zeros((100, 100), np.uint8)
        cells[50:, :] = 254
        grid = CollisionGrid(cells, .025, (1., 2., math.pi/2), 10., 'odom')
        self.assertTrue(grid.safe_segment((.5, 3.), (.4, 3.), .2))
        self.assertFalse(grid.safe_segment((.5, 3.), (-.5, 3.), .2))
        self.assertFalse(grid.safe_segment((1., 2.), (1., 2.), .2))


class TaskTests(unittest.TestCase):
    def test_slow_status_reply_neither_holds_motion_lock_nor_revokes_new_lease(self):
        from navcar_navigation import NavigationBridge
        entered,release=threading.Event(),threading.Event()
        canceled=[]
        panel=NS(lock=threading.RLock(),nav_serial=1,nav_status='active',
                 cancel_navigation=lambda:canceled.append(True))
        def request(path):
            entered.set();release.wait(1.)
            return {'connected':True,'error':'','control_source':'manual'}
        bridge=NS(panel=panel,done=threading.Event(),lease='old',request=request,
                  release=lambda **kwargs:canceled.append('release'))
        worker=threading.Thread(target=NavigationBridge.poll_status,args=(bridge,))
        worker.start()
        try:
            self.assertTrue(entered.wait(1.))
            acquired=panel.lock.acquire(timeout=.1)
            self.assertTrue(acquired,'status HTTP request held the motion state lock')
            if acquired:
                bridge.lease='new';panel.nav_serial=2
                panel.lock.release()
        finally:
            bridge.done.set();release.set();worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertEqual(canceled,[])

    def test_late_status_release_cannot_revoke_a_new_task(self):
        from navcar_navigation import NavigationBridge
        requests=[]
        bridge=NS(io_lock=threading.RLock(),lease='new',panel=NS(nav_serial=2),
                  request=lambda *args:requests.append(args))
        NavigationBridge.release(bridge,expected_lease='old',expected_serial=1)
        self.assertEqual(bridge.lease,'new')
        self.assertEqual(requests,[])
        NavigationBridge.release(bridge,expected_lease='new',expected_serial=1)
        self.assertEqual(bridge.lease,'new')
        NavigationBridge.release(bridge,expected_lease='new',expected_serial=2)
        self.assertIsNone(bridge.lease)
        self.assertEqual(requests,[('/api/navigation/release',{'lease':'new'})])

    def test_slow_map_io_does_not_block_motion_or_commit_an_old_session(self):
        from navcar_navigation import NavigationBridge
        entered,release=threading.Event(),threading.Event()
        applied=[]
        def saved_map():
            entered.set();release.wait(1.)
            return {'revision':'old'},None
        panel=NS(lock=threading.RLock(),saved_map=saved_map,map_pose=None,map_pose_time=0.,
                 navigation_control=NS(running=True,frames={'base_frame':'center','center_offset':[0.,0.,0.]}))
        transform=NS(header=NS(stamp=None),transform=NS(translation=NS(x=1.,y=2.),rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        bridge=NS(panel=panel,grid_epoch=0,client=NS(server_is_ready=lambda:True),
                  tf=NS(lookup_transform=lambda *args:transform),
                  node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=0))),
                  recovery_tick=lambda **kwargs:applied.append(kwargs))
        fake_ros=NS(time=NS(Time=lambda:None))
        with patch.dict('sys.modules',{'rclpy':fake_ros}),patch('navcar_navigation.sample_monotonic_time',return_value=10.):
            worker=threading.Thread(target=NavigationBridge.tick,args=(bridge,))
            worker.start()
            try:
                self.assertTrue(entered.wait(1.))
                acquired=panel.lock.acquire(timeout=.1)
                self.assertTrue(acquired,'slow file I/O held the shared motion state lock')
                if acquired:
                    bridge.grid_epoch+=1
                    panel.lock.release()
            finally:
                release.set();worker.join(1.)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(panel.map_pose)
        self.assertEqual(applied,[])

    def test_grid_worker_discards_old_map_session_and_stops_cleanly(self):
        bridge=NS(done=threading.Event(),grid_condition=threading.Condition(),grid_epoch=0,local_grid=None,
                  panel=NS(lock=threading.RLock(),navigation_control=NS(frames={'collision_radius':.317658})))
        bridge.pending_grid=(0,None,.025,(0.,0.,0.),time.monotonic(),'old')
        entered,release=threading.Event(),threading.Event()
        def build(values,resolution,origin,stamp,frame):
            if frame=='old':
                entered.set();release.wait(1.)
            return NS(stamp=stamp,frame=frame,prepare=lambda radius:None)
        worker=threading.Thread(target=RecoveryBridgeMixin.build_grids,args=(bridge,))
        with patch('navigation_recovery.CollisionGrid',side_effect=build):
            worker.start()
            try:
                self.assertTrue(entered.wait(1.))
                with bridge.grid_condition:
                    bridge.grid_epoch=1
                    bridge.pending_grid=(1,None,.025,(0.,0.,0.),time.monotonic(),'fresh')
                    bridge.grid_condition.notify()
                release.set()
                deadline=time.monotonic()+1.
                while bridge.local_grid is None and time.monotonic()<deadline:time.sleep(.005)
                self.assertIsNotNone(bridge.local_grid)
                self.assertEqual(bridge.local_grid.frame,'fresh')
            finally:
                release.set();bridge.done.set()
                with bridge.grid_condition:bridge.grid_condition.notify_all()
                worker.join(1.)
        self.assertFalse(worker.is_alive())

    def test_real_odom_motion_is_not_a_localization_jump(self):
        first=map_odom_correction({'x':3.,'y':4.,'yaw':0.}, {'x':1.,'y':1.,'yaw':0.})
        moved=map_odom_correction({'x':4.,'y':4.,'yaw':0.}, {'x':2.,'y':1.,'yaw':0.})
        self.assertFalse(correction_jump(first,moved))
        bad=map_odom_correction({'x':5.,'y':4.,'yaw':0.}, {'x':2.,'y':1.,'yaw':0.})
        self.assertTrue(correction_jump(first,bad))
        self.assertFalse(correction_jump((0.,0.,math.pi-.01),(0.,0.,-math.pi+.01)))

    def test_retry_budget_and_hard_collision_never_start_blind_escape(self):
        task = RecoveryTask({'x': 2., 'y': 1.}, 'revision')
        task.failed(105, 1.)
        self.assertEqual(task.state, 'recovering')
        task.failed(205, 2.)
        self.assertEqual(task.state, 'blocked')
        self.assertTrue(task.permanent)
        self.assertEqual(task.goal, {'x': 2., 'y': 1.})
        for i in range(10):
            task.failed(208, float(i+3))
        self.assertEqual(task.state, 'blocked')
        self.assertFalse(task.permanent)

    def test_automatic_cancel_retains_goal_user_cancel_invalidates_late_result(self):
        from test_panel import NavigationTests
        fixture = NavigationTests()
        fixture.setUp()
        try:
            fixture.ready()
            p = fixture.panel
            p.navigation_control=NS(max_speed=.5)
            class Bridge(RecoveryBridgeMixin):
                def __init__(self):
                    self.panel = p
                    self.task = None
                    self.smoother = SmoothVelocity()
                    self.recovery_velocity = (0., 0.)
                def send_goal(self, goal, feedback):
                    return fixture.send(goal, feedback)
            bridge = Bridge()
            p.bridge = bridge
            fixture.submit()
            fixture.response.set_result(fixture.handle)
            original = dict(p.nav_goal)
            bridge.task.state = 'pausing'
            p.cancel_navigation(automatic=True)
            self.assertIsNotNone(bridge.task)
            fixture.result.set_result(NS(status=5, result=NS(error_code=0, error_msg='')))
            self.assertEqual(p.nav_status, 'recovering')
            self.assertEqual(bridge.task.state, 'relocalizing')
            self.assertEqual(p.nav_goal, original)
            old_serial = p.nav_serial
            p.cancel_navigation()
            self.assertIsNone(bridge.task)
            self.assertEqual(p.nav_status, 'canceled')
            late = Future()
            late.set_result(NS(status=4, result=NS(error_code=0)))
            p.navigation_result(old_serial, late)
            self.assertEqual(p.nav_status, 'canceled')
        finally:
            # Fixture closes no real ROS or hardware resources.
            p.bridge = None
            fixture.tearDown()


if __name__ == '__main__':
    unittest.main()

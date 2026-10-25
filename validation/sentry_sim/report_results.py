"""Summarize recorded outcomes without dropping stops or failed scenarios."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite',type=Path,required=True)
    parser.add_argument('--native',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--unit-tests',type=int,required=True)
    parser.add_argument('--geometry',type=Path)
    parser.add_argument('--baseline-suite',type=Path)
    args=parser.parse_args()
    suite=json.loads(args.suite.read_text());native=json.loads(args.native.read_text())
    results=suite['results'];root=args.suite.parent.resolve();args.output.parent.mkdir(parents=True,exist_ok=True)
    labels={'smooth':'常规通行','narrow':'窄道通行','reject':'窄道拒绝','stuck':'受困恢复',
            'localization':'定位故障恢复','cancel':'取消任务'}
    rows=[]
    for r in results:
        motion=r.get('motion',{})
        rows.append(f"| [{r['name']}]({root/r['name']/'result.json'}) | {r['width_m']*100:g} | "
                    f"{'通过' if r['passed'] else '失败'} | {r.get('status','—')} | "
                    f"{motion.get('normal_max_acceleration_mps2',0):.3f} | "
                    f"{motion.get('normal_max_jerk_mps3',0):.3f} | {len(r.get('safety_stops',[]))} | "
                    f"{motion.get('max_jerk_mps3',0):.3f} |")
    normal=[r for r in results if r.get('case') in ('smooth','narrow')]
    stop_count=sum(len(r.get('safety_stops',[])) for r in normal)
    uninterrupted=('本轮常规通行场景未观察到急停记录。' if stop_count==0 else
                   f'本轮常规通行场景仍有 **{stop_count} 次急停记录**。正常行驶段平顺性与整段无停顿是两个判据，整段无停顿尚未通过。')
    failed=[r['name'] for r in results if not r['passed']]
    passed_widths=sorted({r['width_m']*100 for r in normal if r['passed']})
    reject_widths=sorted({r['width_m']*100 for r in results if r['case']=='reject'})
    passage_widths='/'.join(f'{w:g}' for w in passed_widths) or '无'
    refusal_widths='/'.join(f'{w:g}' for w in reject_widths) or '无'
    geometry=json.loads(args.geometry.read_text()) if args.geometry else None
    geometry_text=(f"- [独立碰撞几何对照]({args.geometry.resolve()})：{geometry['checks']} 个临界样例、"
                   f"{geometry['rotated_origins']} 种地图旋转，{'通过' if geometry['passed'] else '失败'}。\n" if geometry else '')
    passage_rows=[]
    for r in normal:
        p=r.get('passage')
        if p:
            passage_rows.append(f"| {r['name']} | {p['max_center_offset_m']*1000:.2f} | "
                                f"{p['max_body_speed_mps']:.3f} | {p['min_physical_clearance_m']*1000:.2f} |")
    comparison=''
    if args.baseline_suite:
        old={r['name']:r for r in json.loads(args.baseline_suite.read_text())['results']}
        shared=[r for r in normal if r['name'] in old]
        before=sum(len(old[r['name']].get('safety_stops',[])) for r in shared)
        after=sum(len(r.get('safety_stops',[])) for r in shared)
        comparison=(f"与 [线程调整前的同一套通行场景]({args.baseline_suite.resolve()}) 对照，"
                    f"{len(shared)} 个相同用例的保护性停车从 **{before} 次降为 {after} 次**。"
                    "这是本轮实测对照，不能保证所有机器负载下都不会超时。\n")
    timing_rows=[]
    def p99(values):
        values=sorted(values)
        return values[min(len(values)-1,int(.99*len(values)))] if values else 0.
    for r in normal:
        directory=root/r['name']
        callbacks=directory/'callback-timing.csv'
        if not callbacks.is_file():continue
        commands=[[float(x) for x in row] for row in csv.reader((directory/'commands.csv').open())]
        gaps=[b[0]-a[0] for a,b in zip(commands,commands[1:]) if math.hypot(a[1],a[2])>.005]
        checked=[float(row[2]) for row in csv.reader(callbacks.open()) if row[1]=='checked_velocity']
        wire=[[float(x) for x in row] for row in csv.reader((directory/'timing.csv').open())]
        waits=[row[1]-row[0] for row in wire]
        timing_rows.append(f"| {r['name']} | {max(gaps,default=0.)*1000:.2f} | "
                           f"{p99(checked)*1000:.2f} | {p99(waits)*1000:.2f} |")
    timing_text=('移动时按模拟 MCU 收到命令的实际时间计量，未放宽超时门槛：\n\n'
                 '| 场景 | 最大运动指令间隔 ms | 输出检查 p99 ms | 状态锁等待 p99 ms |\n'
                 '|---|---:|---:|---:|\n'+'\n'.join(timing_rows)+'\n') if timing_rows else ''
    delayed=[r for r in results if r.get('state_delay_s',0)>0]
    delay_text='\n'.join(f"`{r['name']}`：每次 MCU 状态回复延迟 {r['state_delay_s']*1000:g} ms，"
                         f"{'通过' if r['passed'] else '失败'}，记录 {len(r.get('safety_stops',[]))} 次保护性停车。" for r in delayed)
    text=f'''# 导航恢复与平顺性验证：2026-10-25

测试对象：`chenkl112/shaoshao`，基线 `b73911143913d7f4ff42d3fffcd1d6b85ffd5d5f`，叠加当前工作区修改。
场景来源：`zzt-180/sentry_planning@b9349d2ada3fe0299ccae9d1e483d5f3b79b1cc8` 自带的 `room.world`。

## 结果

{len(results)} 个场景的功能及正常行驶段判据：**{'全部通过' if not failed else '存在失败：'+', '.join(failed)}**。
通行场景已通过宽度：{passage_widths} cm；受困后原目标续行、错误地图位姿触发重定位并续行，以及取消撤销恢复均按各自结果记录。
{refusal_widths} cm 拒绝场景验证的是 **拒绝进入且保留目标**，不是通行成功。

| 场景 / 原始结果 | 宽度 cm | 场景判据 | 结束状态 | 正常段最大加速度 m/s² | 正常段最大 jerk m/s³ | 急停记录 | 全程最大 jerk m/s³ |
|---|---:|---|---|---:|---:|---:|---:|
{chr(10).join(rows)}

瓶颈内真实运动（x=1.3～2.7 m，不含入口/出口的转接段）：

| 场景 | 最大中心偏差 mm | 最大实际速度 m/s | 最小本体至墙净空 mm |
|---|---:|---:|---:|
{chr(10).join(passage_rows)}

{uninterrupted}

{comparison}

{timing_text}

{delay_text}

完整 CSV 与曲线保留了全部急停峰值；正常段统计仅排除明确记录的急停附近 40 ms。
不能用正常段数值宣称整段始终满足舒适性限制。

## 实现

- Smac 2D A* 替代 NavFn，处理本场景中“势场可达但无法提取路径”的窄道失败。
- 局部代价图融合已保存静态地图和实时观测，保留真实未知区，避免单次扫描空洞丢掉已有空闲证据。
- 按用户要求取消额外的每侧 20 mm 余量，Nav2 padding 为零，输出层使用实际本体外接半径 0.297658 m。
- 局部控制栅格为 1 cm 或更细，保存地图和全局地图保持真实分辨率；近障时精确检查圆形扫掠与完整占据/未知方格。
- 前方 0.7 m 检查窄道，平顺降低速度目标至约 0.08 m/s；全局代价引导和双侧障碍横向修正共同促进居中。
- 外层持有原目标、地图版本及动作编号；可恢复失败不会直接丢掉任务。
- 无进展触发受控退让与安全点搜索，动作结束先平稳减速，再重新规划。取消、接管、换图使旧任务失效。
- AMCL 保持唯一地图 TF 权限。协方差、采样时间、独立扫描匹配及地图校正跳变共同判断定位；最后可信校正结合局部里程计生成重定位初值。
- 初始化与重定位至少三帧独立扫描、约一秒稳定确认；无效或重复观测不放行。
- 正常输出目标：加速度 0.25 m/s²、jerk 0.6 m/s³；硬合速度上限 0.5 m/s 时，平顺速度目标至多 0.4 m/s。
- 平顺后的实际指令和实测速度都检查完整车体包络及制动通路，保留完整硬障碍和未知方格，不修改保存地图或实际本体尺寸。
- 代价图预计算移出 ROS 回调线程，旧地图会话的结果不会覆盖当前会话。
- 不同 ROS 回调组在 4 个执行线程中并发运行；最新帧队列避免里程计积压，地图可视化改为 NumPy 向量运算。
- TF 查询与地图读取移出共享状态锁；地图会话代次阻止并发旧回调写回。MCU 状态查询独立于运动输出。
- 迟到的状态回复和租约释放不能撤销新任务；180/300/500 ms 的输出、里程计和局部地图门槛保持不变。
- FAST-LIO 输出滤波速度和当前帧协方差，补偿到中心参考点，取消异步位置差分生成控制速度。
- PNG 导出先完成临时文件写入，再原子替换图像并更新 YAML。

## 其他验证

- Python 回归：{args.unit_tests} 项通过。
{geometry_text}- FAST-LIO：ROS 2 Jazzy 原生构建通过；本轮未修改 C++，沿用此前同一源码的构建和探针记录。
- [新 FAST-LIO 传感器探针结果]({args.native.resolve()})：输入 0.08 m/s，估计均值
  {native.get('mean_forward_mps',0):.6f} m/s，标准差 {native.get('velocity_std_mps',0):.6f} m/s，
  {native.get('observations',0)} 个稳态观测，速度协方差检查{'通过' if native['passed'] else '失败'}。

## 适用边界

Gazebo Harmonic 保留原房间的墙体碰撞几何，替换为本车圆形包络测试体，并增加通道障碍。
使用真实 Nav2、AMCL、任务桥接和输出门控；扫描及局部里程计由仿真几何与物理运动生成。
这没有运行原 ROS 1 OCS2 控制器、完整 RMUC 场地或真实电机。
FAST-LIO 另用合成 MID360/IMU 独立验证，未在 Gazebo 内做完整雷达渲染闭环。
重定位场景验证了连续局部里程计可用时的自动恢复；任意绑架位姿及复杂大场景的全局搜索尚未验收。

圆形包络半径 0.297658 m，取消额外余量后直径约 0.5953 m。
通行结果没有通过缩小实际本体或放开未知区域获得；不能据此推断所有大于外接直径的通道均能通过。
旧压缩包中的稀疏空闲区与空 PNG 仍需修复或重新采集，不会自动被填白。
真实车辆的制动减速度、延迟与轮胎响应仍需低速标定；仿真结果不代表实车验收。

## 复现

见仓库 `validation/sentry_sim/README.md`。本轮依赖为临时目录中的 ROS 2 Jazzy / Gazebo Harmonic overlay，
未连接真实车辆。逐场景源文件 SHA256 存在各自 `result.json`，所有历史失败记录仍在上级证据目录。
'''
    args.output.write_text(text,encoding='utf-8')
    summary={'suite_passed':suite['passed'],'native_probe_passed':native['passed'],'unit_tests_passed':args.unit_tests,
             'uninterrupted_normal_motion':stop_count==0,'normal_safety_stops':stop_count,
             'evidence_root':str(root),'scenarios':[
                 {k:r[k] for k in ['name','case','width_m','start_offset_m','state_delay_s','passed','status','elapsed_s','passage','motion','source_sha256'] if k in r}
                 for r in results]}
    args.output.with_suffix('.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(args.output)

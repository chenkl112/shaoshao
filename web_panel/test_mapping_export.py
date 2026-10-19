"""Check the exported PNG/YAML through Nav2's trinary loading semantics."""
import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace as NS
import unittest
import zlib


spec=importlib.util.spec_from_file_location('lidar_mapping_web_export',
    Path(__file__).resolve().parents[1]/'deployment/lidar_mapping_web.py')
export=importlib.util.module_from_spec(spec)
spec.loader.exec_module(export)


def message(data,width,height):
    return NS(data=data,info=NS(width=width,height=height,resolution=.05,
        origin=NS(position=NS(x=-1.,y=2.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))


class MappingExportTests(unittest.TestCase):
    def test_png_yaml_roundtrip_preserves_unknown_free_obstacle_and_row_order(self):
        msg=message([-1,0,100,100,0,-1],3,2)
        png=export.encode_grid(msg)
        self.assertEqual(png[:8],b'\x89PNG\r\n\x1a\n')
        offset=8;compressed=b''
        while offset<len(png):
            size=struct.unpack('>I',png[offset:offset+4])[0]
            kind=png[offset+4:offset+8];data=png[offset+8:offset+8+size]
            if kind==b'IDAT':compressed+=data
            offset+=size+12
        raw=zlib.decompress(compressed)
        # PNG rows are top-first, occupancy rows are bottom-first.
        self.assertEqual(raw,bytes([0,0,254,205,0,205,254,0]))
        metadata=export.grid_metadata(msg,'live-grid-test.png')
        rows=[raw[i+1:i+4] for i in range(0,len(raw),4)]
        loaded=[]
        for row in rows[::-1]:
            for pixel in row:
                occupancy=(255-pixel)/255.
                loaded.append(100 if occupancy>metadata['occupied_thresh'] else
                              0 if occupancy<metadata['free_thresh'] else -1)
        self.assertEqual(loaded,msg.data)
        self.assertEqual(metadata['origin'],[-1.,2.,0.])
        self.assertEqual(metadata['resolution'],.05)
        # Reproduce the field export bug: gray 205 became free at threshold .25.
        self.assertLess((255-205)/255.,.25)
        self.assertGreater((255-205)/255.,metadata['free_thresh'])

    def test_unobserved_map_is_not_exported_as_an_empty_room(self):
        self.assertIsNone(export.encode_grid(message([-1]*6,3,2)))


if __name__=='__main__':unittest.main()

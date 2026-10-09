"""Disk replay must preserve the pixels used by the live image observer."""
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from .recording import frame_sha256,open_lossless_writer


class MonoRecording(unittest.TestCase):
    def test_native_pixels_survive_disk_codec_roundtrip(self):
        rng=np.random.default_rng(42)
        frames=[rng.integers(0,256,(320,320),dtype=np.uint8),np.zeros((320,320),dtype=np.uint8),
                ((np.indices((320,320)).sum(axis=0)%2)*255).astype(np.uint8)]
        with tempfile.TemporaryDirectory() as td:
            path=Path(td)/'camera.avi';writer=open_lossless_writer(path,20.)
            hashes=[frame_sha256(frame) for frame in frames]
            for frame in frames:writer.write(frame)
            writer.release();reader=cv2.VideoCapture(str(path))
            for expected in hashes:
                ok,bgr=reader.read();self.assertTrue(ok)
                np.testing.assert_array_equal(bgr[:,:,0],bgr[:,:,1])
                np.testing.assert_array_equal(bgr[:,:,1],bgr[:,:,2])
                self.assertEqual(frame_sha256(bgr[:,:,0]),expected)
            self.assertFalse(reader.read()[0]);reader.release()

    def test_invalid_native_contract_is_rejected(self):
        for gray in (np.zeros((324,244),dtype=np.uint8),np.zeros((320,320,3),dtype=np.uint8),
                     np.zeros((320,320),dtype=np.float32)):
            with self.assertRaises(ValueError):frame_sha256(gray)


if __name__=='__main__':unittest.main()

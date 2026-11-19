"""Foreground/neighbor isolation and native-coordinate RLE, with no training."""
from pathlib import Path
from tempfile import TemporaryDirectory
import importlib.util
import numpy as np
from PIL import Image,ImageDraw
spec=importlib.util.spec_from_file_location('contour',Path(__file__).with_name('contour.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
with TemporaryDirectory() as directory:
    path=Path(directory)/'sample.png';image=Image.new('RGB',(640,480),(25,30,35));draw=ImageDraw.Draw(image);draw.ellipse((220,130,400,350),fill=(195,158,111));draw.rectangle((460,160,570,310),fill=(195,158,111));image.save(path)
    truth=Image.new('L',image.size);ImageDraw.Draw(truth).ellipse((220,130,400,350),fill=1);truth=np.array(truth)>0
    result=m.propose_contour(path,{'bbox_xyxy':[200,110,420,370]},dict(max_processing_edge_px=1600,iterations=3,threads=2,seed_radius_px=3));mask=m.decode_rle(result['mask'],640,480)>0
    iou=(mask&truth).sum()/(mask|truth).sum();assert iou>.98,iou
    assert not mask[160:311,460:571].any()
    assert np.array_equal(mask,m.decode_rle(m.encode_rle(mask),640,480))
    assert result['approved'] is False
    try:m.propose_contour(path,{'bbox_xyxy':[-1,0,5,5]},dict(max_processing_edge_px=1600,iterations=3,threads=2,seed_radius_px=3))
    except ValueError:pass
    else:raise AssertionError('Invalid box accepted')
    print({'native_size':result['mask']['size'],'isolated_target_iou':float(iou),'elapsed_ms':result['elapsed_ms'],'training_started':False})

    # Inspect the actual GrabCut initialization on a repeat request: the old mask
    # must partition the box into foreground/background, preserving its holes.
    original=m.cv2.grabCut;seen={}
    def capture(pixels,mask,*args):seen['initial']=mask.copy();return original(pixels,mask,*args)
    m.cv2.grabCut=capture
    try:
        prompt={'bbox_xyxy':[200,110,420,370],'initial_mask':result['mask'],'negative_points':[[210,120]]}
        refined=m.propose_contour(path,prompt,dict(max_processing_edge_px=1600,iterations=3,threads=2,seed_radius_px=3))
        assert seen['initial'][120,210]==m.cv2.GC_BGD
        assert seen['initial'][115,215]==m.cv2.GC_PR_BGD
        assert seen['initial'][240,310]==m.cv2.GC_PR_FGD
        assert seen['initial'][100,190]==m.cv2.GC_BGD
        output=m.decode_rle(refined['mask'],640,480)>0
        assert (output&truth).sum()/(output|truth).sum()>.98
        print({'iterative_partition':'passed','negative_seed':'passed'})
    finally:m.cv2.grabCut=original

"""Box/point guided GrabCut suggestions, independent of class and ROS."""
import math,time
import cv2
import numpy as np
from PIL import Image

def decode_rle(rle,width,height):
    counts=rle.get('counts',[])
    if rle.get('size')!=[height,width] or not counts or not all(type(v) is int and v>=0 for v in counts) or sum(counts)!=width*height:
        raise ValueError('Invalid original-pixel RLE')
    result=np.zeros(width*height,np.uint8);offset=0
    for i,n in enumerate(counts):
        if i%2:result[offset:offset+n]=1
        offset+=n
    return result.reshape((height,width),order='F')

def encode_rle(mask):
    flat=np.asarray(mask,dtype=np.uint8).ravel(order='F')
    changes=np.flatnonzero(flat[1:]!=flat[:-1])+1
    runs=np.diff(np.r_[0,changes,len(flat)]).tolist()
    if flat[0]:runs.insert(0,0)
    return {'size':list(mask.shape),'counts':runs}

def propose_contour(path,prompt,config):
    started=time.monotonic();rgb=np.asarray(Image.open(path).convert('RGB'));height,width=rgb.shape[:2]
    scale=min(1.,config['max_processing_edge_px']/max(width,height));w=max(2,round(width*scale));h=max(2,round(height*scale))
    pixels=cv2.resize(rgb,(w,h),interpolation=cv2.INTER_AREA) if scale<1 else rgb.copy()
    def point(p):
        if not isinstance(p,list) or len(p)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in p) or not (0<=p[0]<width and 0<=p[1]<height):raise ValueError('Point must be inside the original image')
        return (min(w-1,round(p[0]*w/width)),min(h-1,round(p[1]*h/height)))
    mask=np.full((h,w),cv2.GC_BGD,np.uint8)
    bbox=prompt.get('bbox_xyxy')
    if bbox is not None:
        if not isinstance(bbox,list) or len(bbox)!=4 or any(type(v) not in (int,float) or not math.isfinite(v) for v in bbox):raise ValueError('Draw a box around the target')
        x1,y1,x2,y2=bbox
        if not (0<=x1<x2<=width and 0<=y1<y2<=height):raise ValueError('Box must fit inside the original image')
        x1=max(0,int(x1*w/width));y1=max(0,int(y1*h/height));x2=min(w,int(math.ceil(x2*w/width)));y2=min(h,int(math.ceil(y2*h/height)))
        if x2-x1<3 or y2-y1<3:raise ValueError('Target box is too small')
        mask[y1:y2,x1:x2]=cv2.GC_PR_FGD
    elif 'initial_mask' not in prompt:
        raise ValueError('Draw a rough box around the target first')
    if 'initial_mask' in prompt:
        initial=decode_rle(prompt['initial_mask'],width,height)
        initial=cv2.resize(initial,(w,h),interpolation=cv2.INTER_NEAREST)
        if bbox is None:
            mask[:]=cv2.GC_PR_BGD
            mask[initial>0]=cv2.GC_PR_FGD
        else:
            # Refine the previous partition instead of resetting the entire box to foreground.
            mask[y1:y2,x1:x2]=cv2.GC_PR_BGD
            region=mask[y1:y2,x1:x2]
            region[initial[y1:y2,x1:x2]>0]=cv2.GC_PR_FGD
    for key,value in [('positive_points',cv2.GC_FGD),('negative_points',cv2.GC_BGD)]:
        points=prompt.get(key,[])
        if not isinstance(points,list) or len(points)>128:raise ValueError('Too many correction points')
        for p in points:cv2.circle(mask,point(p),max(1,config['seed_radius_px']),value,-1)
    if not np.any((mask==cv2.GC_BGD)|(mask==cv2.GC_PR_BGD)) or not np.any((mask==cv2.GC_FGD)|(mask==cv2.GC_PR_FGD)):
        raise ValueError('Leave a little background outside the target box')
    cv2.setNumThreads(config['threads'])
    cv2.grabCut(pixels,mask,None,np.zeros((1,65),np.float64),np.zeros((1,65),np.float64),config['iterations'],cv2.GC_INIT_WITH_MASK)
    foreground=np.isin(mask,[cv2.GC_FGD,cv2.GC_PR_FGD]).astype(np.uint8)
    if not foreground.any():raise ValueError('No contour found; redraw the box or add an Include point')
    if scale<1:foreground=cv2.resize(foreground,(width,height),interpolation=cv2.INTER_NEAREST)
    return {'mask':encode_rle(foreground),'approved':False,'method':'grabcut','elapsed_ms':round((time.monotonic()-started)*1000,2)}

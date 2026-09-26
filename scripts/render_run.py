"""Draw a local replay of ACTUAL recorded odometry; not a Foxglove screen capture.
Requires Pillow. Run: python scripts/render_run.py
"""
from pathlib import Path
import json
import math
from PIL import Image, ImageDraw, ImageFont

root = Path(__file__).resolve().parents[1]
run = json.loads((root / 'evidence/navigation-run.json').read_text())
samples = run['samples']
boxes = [(7,-6,3,3),(-8,6,3,3),(4,9,3,3),(9,3,2,2),(1,-10,2,2)]
circles = [(0,0,3),(-7,-7,2)]

def clearance(p):
    x,y=p['x'],p['y']
    distances=[14.75-abs(x),14.75-abs(y)]
    distances += [math.hypot(x-cx,y-cy)-r for cx,cy,r in circles]
    distances += [math.hypot(max(abs(x-cx)-w/2,0),max(abs(y-cy)-h/2,0)) for cx,cy,w,h in boxes]
    return min(distances)

summary = {'sample_count':len(samples),'duration_seconds':samples[-1]['t']-samples[0]['t'],
           'minimum_reference_point_clearance_m':min(map(clearance,samples)),
           'conservative_body_radius_m':math.hypot(1.5,0.5),
           'last_command':[samples[-1]['v'],samples[-1]['w']],
           'note':'Sampled geometric check against the static world; not a contact-sensor collision log.'}
summary['minimum_bounding_circle_margin_m']=summary['minimum_reference_point_clearance_m']-summary['conservative_body_radius_m']
(root/'evidence/trajectory-summary.json').write_text(json.dumps(summary,indent=2)+'\n')

try:
    font=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',19)
    small=ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf',15)
    title=ImageFont.truetype('C:/Windows/Fonts/segoeuib.ttf',26)
except OSError:
    font=small=title=ImageFont.load_default()

def xy(x,y):return (60+(x+16)*20,95+(16-y)*20)
base=Image.new('RGB',(760,820),'#f6f7fa')
d=ImageDraw.Draw(base)
d.text((40,20),'Three autonomous trips through Gazebo',font=title,fill='#17263b')
d.text((40,55),'Replay of recorded robot positions | world coordinates in metres',font=small,fill='#44556b')
for a in range(-15,16,5):
    d.line([xy(a,-15),xy(a,15)],fill='#dbe1e9')
    d.line([xy(-15,a),xy(15,a)],fill='#dbe1e9')
    d.text((xy(a,-16)[0]-8,xy(a,-16)[1]+3),str(a),font=small,fill='#44556b')
    d.text((18,xy(-16,a)[1]-8),str(a),font=small,fill='#44556b')
d.rectangle([xy(-15,15),xy(15,-15)],outline='#627184',width=5)
for x,y,w,h in boxes:d.rectangle([xy(x-w/2,y+h/2),xy(x+w/2,y-h/2)],fill='#a7b0bc',outline='#627184',width=2)
for x,y,r in circles:d.ellipse([xy(x-r,y+r),xy(x+r,y-r)],fill='#a7b0bc',outline='#627184',width=2)
for i,result in enumerate(run['results'],1):
    x,y=xy(*result['goal']);d.ellipse((x-7,y-7,x+7,y+7),fill='#18835f')
    d.text((x+10,y-12),str(i),font=font,fill='#126148')
d.text((40,770),'Grey: fixed obstacles   Green: goals   Blue: actual travelled route',font=small,fill='#44556b')
d.text((40,795),'This is a telemetry replay, not a Foxglove recording.',font=small,fill='#44556b')

def frame(index, annotate=False):
    image=base.copy();draw=ImageDraw.Draw(image)
    if index>0:draw.line([xy(s['x'],s['y']) for s in samples[:index+1]],fill='#2867c5',width=3)
    p=samples[index];x,y=xy(p['x'],p['y']);yaw=p['yaw']
    # Robot footprint from the model reference: chassis centre is 0.5 m ahead.
    corners=[]
    for a,b in [(-0.5,-0.5),(1.5,-0.5),(1.5,0.5),(-0.5,0.5)]:
        corners.append(xy(p['x']+math.cos(yaw)*a-math.sin(yaw)*b,p['y']+math.sin(yaw)*a+math.cos(yaw)*b))
    draw.polygon(corners,fill='#2867c5',outline='#153968')
    draw.line([(x,y),xy(p['x']+1.7*math.cos(yaw),p['y']+1.7*math.sin(yaw))],fill='#fff',width=2)
    draw.text((570,78),f"t = {p['t']-samples[0]['t']:.1f} s",font=small,fill='#17263b')
    return image

frame(len(samples)-1).save(root/'evidence/actual-trajectory.png')
indices=list(range(0,len(samples),10))
if indices[-1]!=len(samples)-1:indices.append(len(samples)-1)
frames=[frame(i) for i in indices]
durations=[max(20,int((samples[b]['t']-samples[a]['t'])*100)) for a,b in zip(indices,indices[1:])]+[1500]
frames[0].save(root/'evidence/telemetry-replay.gif',save_all=True,append_images=frames[1:],duration=durations,loop=0,optimize=True)
print(json.dumps(summary,indent=2))

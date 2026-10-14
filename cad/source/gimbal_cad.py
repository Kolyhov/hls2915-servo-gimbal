"""HLS2915 ceiling camera head, concept revision A. Units mm.
No motor-control code. Geometry only. Hardware interfaces require fit coupons.
Assembly coordinates: +Z down from ceiling, camera looks +Y.
"""
import math
import cadquery as cq

def box(x0,x1,y0,y1,z0,z1):
    return cq.Workplane('XY').box(x1-x0,y1-y0,z1-z0).translate(((x0+x1)/2,(y0+y1)/2,(z0+z1)/2))

def cyl(r,h,x=0,y=0,z=0):
    return cq.Workplane('XY').circle(r).extrude(h).translate((x,y,z))

def hexhole(af,h,x,y,z):
    return cq.Workplane('XY').polygon(6,af/math.cos(math.pi/6)).extrude(h).translate((x,y,z))

def fuse(*shapes):
    a=shapes[0]
    for b in shapes[1:]: a=a.union(b)
    return a

BOLTS=[(x,y) for x in (-27,12) for y in (-14,14)]
YAW=(0,0,25)
PITCH=(-43,-12,82)
ROLL=(11,-20,82)

def place(s,which):
    if which=='yaw': return s.translate(YAW)
    if which=='pitch':
        # local X->world Z, local Y->-Y, local Z->world X
        return s.rotate((0,0,0),(1,0,1),180).translate(PITCH)
    if which=='roll':
        return s.rotate((0,0,0),(1,0,0),-90).translate(ROLL)
    raise ValueError(which)

def clamp_half(front=False):
    a=box(-31,16,-18,18,-15.3,-.15)
    a=a.edges('|Z').fillet(1.3)
    a=a.cut(box(-24.75,9.75,-10.25,10.25,-11.75,1))
    a=a.cut(cyl(10.4,18,z=-16))
    # Opposed connector windows. Open half-seams admit preterminated leads.
    for ya,yb in [(-19,-9),(9,19)]:
        a=a.cut(box(-13,5.5,ya,yb,-12,1))
    # Tail window to reduce mass, preserve perimeter and torsion ribs.
    a=a.cut(box(-22.5,-13.5,-6.5,6.5,-16,-10.8))
    for x,y in BOLTS:
        a=a.cut(cyl(1.45,18,x,y,-16))
        a=a.cut(hexhole(5.2,2.8,x,y,-15.4))
    if front:
        a=a.mirror('XY',union=False)
        # The front has round head seats instead of exposed hex nut seats.
        for x,y in BOLTS:
            a=a.cut(cyl(2.75,3.0,x,y,12.6))
    return a

def horn_adapter():
    # Contact boss reaches through fixed clamp opening without rubbing.
    a=fuse(cyl(9.6,1.8,z=14.5),cyl(12,3.2,z=16.3))
    a=a.cut(cyl(3.25,7,z=14))
    for deg in (45,135,225,315):
        x,y=6*math.cos(math.radians(deg)),6*math.sin(math.radians(deg))
        a=a.cut(cyl(1.1,7,x,y,14))
        a=a.cut(cyl(2.1,2.01,x,y,17.5))
    return a

def final_horn_holes(shape,axis=None):
    cutters=[cyl(3.25,20,z=10)]
    for deg in (45,135,225,315):
        x,y=6*math.cos(math.radians(deg)),6*math.sin(math.radians(deg))
        cutters += [cyl(1.1,16,x,y,14),cyl(2.1,12.5,x,y,17.5)]
    for cutter in cutters:
        shape=shape.cut(place(cutter,axis) if axis else cutter)
    return shape

def tie_slot_pair(shape,x0,x1,y0,y1,z0,z1):
    # Generic through slots for 2.5 mm cable ties; no snap-latch fatigue.
    for y in (y0,y1-1.6):
        shape=shape.cut(box(x0,x1,y,y+1.6,z0,z1))
    return shape

def usb_saddle():
    # Open U channel,11x8clear; M2.5 cap secures cable via soft liner.
    a=box(-13,13,-7,7,0,11)
    a=a.cut(box(-5.5,5.5,-8,8,3,12))
    for x in (-9,9):
        a=a.cut(cyl(1.45,13,x,0,-1))
        a=a.cut(hexhole(5.2,2.7,x,0,-.1))
    return a

def usb_cap():
    a=box(-13,13,-7,7,0,5.4)
    for x in (-9,9): a=a.cut(cyl(1.45,7,x,0,-.5))
    return a

def servo_reference():
    body=box(-24.5,9.5,-10,10,-11.5,11.5)
    horn=fuse(cyl(7.995,3,z=11.5),cyl(7.995,3,z=-14.5))
    return body,horn

def build_gimbal():
    rear,front=clamp_half(),clamp_half(True)
    plate=box(-38,34,-29,29,0,4).edges('|Z').fillet(4)
    for x in (-30,26):
        for y in (-23,23):
            plate=plate.cut(cyl(2.25,6,x,y,-1)).cut(cyl(4.5,2.6,x,y,1.5))
    for y in (-7,7): plate=plate.cut(cyl(1.7,6,29,y,-1))
    # Two side webs leave nuts accessible before ceiling attachment.
    base=fuse(plate,place(rear,'yaw'))
    for y0,y1 in [(-18,-12),(12,18)]:
        base=base.union(box(-21,-13,y0,y1,3.5,12))
    # Stationary USB guide, wide open to side; uses separate cap.
    saddle0=usb_saddle().translate((0,38,0))
    base=base.union(box(-13,13,27,33,0,4)).union(saddle0)
    # Yaw carrier combines driven flange, beam and pitch rear clamp.
    yawcarrier=place(horn_adapter(),'yaw')
    beam=cq.Workplane('XY').polyline([(-7,9),(-59,-3),(-59,-23),(-43,-23),(8,-8)]).close().extrude(4).translate((0,0,41.3))
    yawcarrier=fuse(yawcarrier,beam,place(rear,'pitch'),box(-58.3,-51,-24,0,43,57.0))
    # Open servo-wire channel across upper beam, 7 mm internal width.
    channel=box(-43,-12,-15,-4,44.5,50).cut(box(-44,-11,-13,-6,46,51))
    yawcarrier=yawcarrier.union(channel)
    # Tie loops terminate the channel at both ends.
    for x in (-38,-18):
        yawcarrier=yawcarrier.cut(box(x,x+3,-16,-3,43.5,45.5))
    yawcarrier=final_horn_holes(yawcarrier,'yaw')
    # Pitch carrier is L-shaped, fixed to roll servo tail via broad rear wall.
    pitchcarrier=fuse(place(horn_adapter(),'pitch'),place(rear,'roll'),box(-26.7,-23.5,-35.3,-11,74,90),box(-26.7,-17,-35.3,-20.3,74,90))
    # Relieve material where the central output screw is reached.
    pitchcarrier=pitchcarrier.cut(cq.Workplane('YZ').circle(3.3).extrude(15).translate((-31,-12,82)))
    # Additional shallow servo cable trough on rear roll side, clear of case.
    trough=box(-20,18,-40,-34.8,72,83).cut(box(-21,19,-40.5,-36.5,74,81))
    pitchcarrier=pitchcarrier.union(trough)
    for x in (-14,11):
        pitchcarrier=pitchcarrier.cut(box(x,x+3,-40.5,-34.5,72,74))
    pitchcarrier=final_horn_holes(pitchcarrier,'pitch')
    # Deep camera frame for stiff7mm USB: PCB at53.3;37mm rear space.
    frame=box(-22,22,-22,22,16.3,19.5).cut(box(-16,16,-16,16,16,20))
    # Lower connector notch (world camera bottom is local negative Y).
    frame=frame.cut(box(-7,7,-23,-14,16,20))
    cam=horn_adapter().union(frame)
    for sx in (-1,1):
        for sy in (-1,1):
            # Four diagonal spokes attach hub to corner mounting bosses.
            spoke=cq.Workplane('XY').polyline([(sx*4,sy*2),(sx*19,sy*16),(sx*16,sy*19),(sx*2,sy*4)]).close().extrude(3.2).translate((0,0,16.3))
            cam=cam.union(spoke).union(cyl(3.7,37,sx*17,sy*17,16.3))
            cam=cam.cut(cyl(1.2,8,sx*17,sy*17,47))
            cam=cam.cut(hexhole(4.3,2.2,sx*17,sy*17,49.0))
            # Side-load the M2 nuts;2.1mm of plastic remains above the pocket.
            xx0,xx1=(sx*17,23) if sx>0 else(-23,sx*17)
            cam=cam.cut(box(xx0,xx1,sy*17-2.2,sy*17+2.2,49.0,51.2))
    # Windowed side webs brace the long standoffs without enclosing the cable.
    for xa,xb in [(-22,-19),(19,22)]:
        wall=box(xa,xb,-18,18,16.3,53.3).cut(box(xa-.2,xb+.2,-10,10,23,46))
        # Keep the lateral nut entrances unobstructed after the union.
        for sy in (-1,1):wall=wall.cut(box(xa-.2,xb+.2,sy*17-2.2,sy*17+2.2,49,51.2))
        cam=cam.union(wall)
    # Flat cable saddle behind lower left of the camera, detachable clamp cap.
    # It sits below PCB and routes cable rearward without hiding connector.
    cam=cam.union(box(-22,-16,-59,-19,16.3,22.7)).union(box(16,22,-59,-19,16.3,22.7))
    saddle_cam=usb_saddle().rotate((0,0,0),(0,1,0),180).translate((0,-61,19.5))
    cam=cam.union(box(-22,22,-59,-54,16.3,22.7)).union(saddle_cam)
    cap_cam=usb_cap().rotate((0,0,0),(0,1,0),180).translate((0,-61,8.5))
    camera=place(final_horn_holes(cam),'roll')
    # Part records hold assembled coordinates; final exporter orients for printing.
    parts=[
      dict(id='G01',name='Ceiling_mount',shape=base,color='#354a5f',print_rot=None,qty=1),
      dict(id='G02',name='Yaw_clamp',shape=place(front,'yaw'),color='#69859b',print_rot=((1,0,0),180),qty=1),
      dict(id='G03',name='Yaw_carrier',shape=yawcarrier,color='#129e9a',print_rot=((1,0,0),90),qty=1),
      dict(id='G04',name='Pitch_clamp',shape=place(front,'pitch'),color='#69859b',print_rot=((0,1,0),90),qty=1),
      dict(id='G05',name='Pitch_carrier',shape=pitchcarrier,color='#1fbbb0',print_rot=None,qty=1),
      dict(id='G06',name='Roll_clamp',shape=place(front,'roll'),color='#69859b',print_rot=((1,0,0),-90),qty=1),
      dict(id='G07',name='Camera_frame',shape=camera,color='#e7a249',print_rot=((0,1,0),90),qty=1),
      dict(id='G08',name='USB_cap',shape=usb_cap().translate((0,38,11)),color='#e7a249',print_rot=None,qty=2),
    ]
    extra=[dict(name='Camera cable clamp cap',shape=place(cap_cam,'roll'),color='#e7a249')]
    refs=[]
    for axis in ('yaw','pitch','roll'):
        body,horns=servo_reference()
        refs += [dict(name=axis+' servo',shape=place(body,axis),color='#242b32'),dict(name=axis+' horns',shape=place(horns,axis),color='#777e87')]
    pcb=box(-19,19,-19,19,53.3,54.9)
    for x in (-17,17):
        for y in (-17,17): pcb=pcb.cut(cyl(1.1,3,x,y,53))
    lens=fuse(cyl(8.5,5,z=54.9),cyl(7,17.4,z=59.9))
    refs += [dict(name='Camera PCB',shape=place(pcb,'roll'),color='#344538'),dict(name='Lens',shape=place(lens,'roll'),color='#171d24'),dict(name='Lens glass',shape=place(cyl(5.5,.25,z=77.3),'roll'),color='#367db3')]
    return dict(parts=parts,refs=refs,extra=extra,axes=dict(yaw=YAW,pitch=PITCH,roll=ROLL))

def fit_coupons():
    camera_pattern=box(-22,22,-22,22,0,2.4).cut(box(-14,14,-14,14,-1,4))
    for x in (-17,17):
        for y in (-17,17): camera_pattern=camera_pattern.cut(cyl(1.2,4,x,y,-.5))
    return [dict(id='T01',name='Servo_half_fit',shape=clamp_half(),qty=1,color='#69859b',print_rot=None),
            dict(id='T02',name='Horn_interface_fit',shape=horn_adapter(),qty=1,color='#e7a249',print_rot=((1,0,0),180)),
            dict(id='T03',name='Camera_hole_fit',shape=camera_pattern,qty=1,color='#69859b',print_rot=None)]

if __name__=='__main__':
    result=build_gimbal()
    for p in result['parts']:
        v=p['shape'].val(); b=v.BoundingBox()
        print(p['id'],p['name'],v.isValid(),len(p['shape'].solids().vals()),round(v.Volume(),1),tuple(round(x,1) for x in (b.xlen,b.ylen,b.zlen)),flush=True)

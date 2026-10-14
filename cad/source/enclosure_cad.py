"""Parametric ceiling enclosure, dimensions in mm.

Base lies against ceiling at z=0; positive z points down in service.  The whole
tub-shaped cover removes downward. Thus the real slide-in rails can be loaded
from their open ends while the cover is absent. No threads are printed.

SOURCE / UNCERTAINTY: URT-2 outline 55.8 x 37.6 (table says36.6); mounting
48.5 x28.7 on drawing versus48.8 x28.8 in table. Slotted supports / floating
M2.5 nuts accommodate this discrepancy. ESP footprint51x21 from screenshot;
PCB1.6, component envelopes and USB position are unmeasured assumptions.
IMU20x15x1 from screenshot; no component envelope provided. All cutouts use
generous open service windows, not asserted connector-fit measurements.
Solder wires on the TOP of both headerless boards, keeping the outer0.65 mm
underside edge clear of solder. Retention lips are interrupted near solder
pads. Check this against actual boards before printing the complete assembly.
"""
from pathlib import Path
import math
import sys

DEPS = Path(__file__).parent / 'pydeps'
if DEPS.exists():
    sys.path.insert(0, str(DEPS))
import cadquery as cq

W, D, H = 90.0, 78.0, 25.0
BASE = 2.8
WALL = 2.4
PCB_Z = 6.0
COVER_Z = BASE
COVER_FLOOR = 22.4
URT_C = (-12.9, 12.0)
ESP_C = (15.7, -21.3)
IMU_C = (-26.5, -25.0)
CASE_SCREWS = [(-37.8, -10.8), (37.0, 29.0), (37.0, -3.0)]
URT_HOLES = [(URT_C[0]+i*24.25,URT_C[1]+j*14.35)
             for i in (-1,1) for j in (-1,1)]

def box(x,y,z,cx=0,cy=0,z0=0):
    return cq.Workplane('XY').box(x,y,z,centered=(True,True,False)).translate((cx,cy,z0))

def cyl(d,h,x=0,y=0,z=0):
    return cq.Workplane('XY').center(x,y).circle(d/2).extrude(h).translate((0,0,z))

def hexprism(flat,h,x=0,y=0,z=0):
    # Horizontal flats y=+/-flat/2, useful for the insertion slots.
    return cq.Workplane('XY').center(x,y).polygon(6,flat/math.cos(math.pi/6)).extrude(h).translate((0,0,z))

def capsule(d,length,h,x=0,y=0,z=0):
    return cq.Workplane('XY').center(x,y).slot2D(length,d).extrude(h).translate((0,0,z))

def join(items):
    a=items[0]
    for b in items[1:]:
        a=a.union(b)
    return a

def board_rails(cx,cy,length,width,thickness,lip_len):
    """Elevated lower ledge, lateral guides, and short corner upper lips."""
    a=[]
    groove = thickness+0.40
    for sign in (-1,1):
        edge=cy+sign*width/2
        # 0.30 lateral clearance; lower ledge engages only0.65mm of edge.
        a.append(box(length+1.05,1.4,PCB_Z+groove+1.2-BASE,
                     cx+.525,edge+sign*1.0,BASE))
        a.append(box(length,2.35,1.2,cx,edge+sign*0.525,PCB_Z-1.2))
        for end in (-1,1):
            a.append(box(lip_len,2.35,1.2,
                         cx+end*(length/2-lip_len/2),edge+sign*0.525,
                         PCB_Z+groove))
        # End stop only at the rail corners, leaving centre / USB clear.
        a.append(box(.8,2.35,groove+1.2,
                     cx+length/2+.65,edge+sign*.525,PCB_Z))
    return join(a)

def build_enclosure():
    # Base + two ceiling ears. Screws remain reachable after cover is fitted.
    base=box(W,D,BASE)
    for x in (-50.0,50.0):
        ear=box(12,16,BASE,x,0,0).edges('|Z').fillet(2.5)
        base=base.union(ear).cut(cyl(4.6,4.0,x,0,-.1))

    # Raised rails: PCB undersides have3.2mm clearance from the base surface.
    base=base.union(board_rails(*ESP_C,51,21,1.6,3.4))
    base=base.union(board_rails(*IMU_C,20,15,1.0,1.2))

    # Retainers use M2.5x8 and nuts loaded into the ceiling face before mounting.
    retainers=[]
    for name,x,y,bw,bl in [('ESP',-12.65,ESP_C[1],4.7,23.4),
                          ('IMU',-39.2,IMU_C[1],4.7,17.4)]:
        base=base.union(cyl(5.0,5.0-BASE,x,y,BASE))
        base=base.cut(hexprism(5.35,2.2,x,y,-.01))
        base=base.cut(cyl(2.85,5.2,x,y,0))
        bar=box(bw,bl,3.2,x,y,5.0).edges('|Z').fillet(.55)
        bar=bar.cut(cyl(2.85,3.4,x,y,4.9))
        retainers.append((name,bar))

    # URT-2: supports with x slots and floating side-loaded M2.5 nut traps.
    for x,y in URT_HOLES:
        p=box(8.8,8.0,PCB_Z-BASE,x,y,BASE)
        pocket=hexprism(5.35,2.25,x-.18,y,2.95).union(
               hexprism(5.35,2.25,x+.18,y,2.95))
        # Slot opens in +Y; insert nut before the PCB.
        pocket=pocket.union(box(5.7,5.0,2.25,x,y+2.5,2.95))
        p=p.cut(pocket).cut(capsule(2.85,3.6,3.5,x,y,2.7))
        base=base.union(p)
        # Screw tip extends below the nut; keep the base clear as well.
        base=base.cut(capsule(2.85,3.6,6.3,x,y,-.1))

    # Three M3 cover posts. Captive nuts slide from +X at z10..12.7.
    for i,(x,y) in enumerate(CASE_SCREWS):
        dep=7.0 if i==0 else 9.0
        p=box(8.3 if i==0 else 9.0,dep,15.0-BASE,x,y,BASE)
        p=p.cut(hexprism(5.8,2.7,x,y,10.0))
        p=p.cut(box(5.2,5.8,2.7,x+2.6,y,10.0))
        p=p.cut(cyl(3.4,6.7,x,y,8.5))
        base=base.union(p)

    # Two independent power cable tie bridges, in the free UART corridor.
    # 2.5mm zip ties fit the2.0x4.0mm under-bridge tunnels.
    for y in (8.0,19.0):
        anchor=box(6,7,4.5,33.0,y,BASE)
        anchor=anchor.cut(box(6.4,3.1,2.0,33.0,y,BASE+.5))
        base=base.union(anchor)

    # Downward cover: sidewalls + bottom as one connected tub.
    cover=box(W,D,H-BASE,z0=BASE).cut(
          box(W-2*WALL,D-2*WALL,COVER_FLOOR-BASE+.1,z0=BASE-.1))
    # Loose mating rim receives the base edge,0.25mm clearance per side.
    # The rim is the ordinary shell wall; base sits flush on its opening.
    # Service apertures open to the ceiling seam for removable cover.
    # USB, ample clearance for an ordinary USB-C cable housing.
    cover=cover.cut(box(5.0,16.0,11.8,44.5,ESP_C[1],BASE-.1))
    # Both TTL G/V1/S sockets (URT is rotated180 degrees in the assembly).
    cover=cover.cut(box(40.0,5.0,17.8,-1.0,38.5,BASE-.1))
    # Terminal strip service access. Also accommodates protruding terminal body.
    cover=cover.cut(box(5.0,28.0,17.2,-44.5,12.0,BASE-.1))
    # Independent 12V and5V pigtail openings,6.5mm width, open to the seam.
    for y in (8.0,19.0):
        cover=cover.cut(box(5.0,6.5,10.5,44.5,y,BASE-.1))
    # Cover floor compression columns meet base posts exactly atz15.
    for i,(x,y) in enumerate(CASE_SCREWS):
        dep=6.5 if i==0 else 8.0
        p=box(8.0,dep,COVER_FLOOR-15.0,x,y,15.0)
        cover=cover.union(p).cut(cyl(3.4,10.3,x,y,14.9))
        # External washer / screw head lands are deliberately flat.

    # Small vent slots away from boards' seating surfaces; floor remains sturdy.
    for x in (-25,-17,-9, -1,7,15):
        cover=cover.cut(capsule(2.0,12.0,3.0,x,23.0,22.2).rotate((x,23,0),(x,23,1),90))
    # Opposing sidewall vents: narrow2mm bridges print without support.
    # Their height avoids the ESP USB aperture and the terminal service window.
    for x in (-44.5,44.5):
        for y in (-30.0,-26.0,-22.0,-18.0):
            cover=cover.cut(box(5.0,2.0,5.0,x,y,15.6))

    # Camera and servo bus connect externally; enclosure has no camera adapter.
    pcb_urt=box(55.8,37.6,1.6,*URT_C,PCB_Z)
    for x,y in URT_HOLES:
        pcb_urt=pcb_urt.cut(cyl(3.0,1.8,x,y,PCB_Z-.1))
    pcb_esp=box(51,21,1.6,*ESP_C,PCB_Z)
    pcb_imu=box(20,15,1.0,*IMU_C,PCB_Z)
    # These are visual envelopes, not supplier-certified electronic CAD.
    usb=box(7.0,8.8,3.2,40.5,ESP_C[1],7.6)
    esp_chips=join([box(8,8,1.5,9,-21.3,7.6),box(8,8,1.5,23,-21.3,7.6)])
    imu_chip=box(5.0,5.0,1.8,*IMU_C,7.0)
    urt_components=join([box(10,9,3,-4,10,7.6),box(8,7,3,-20,17,7.6),
                         box(7,24,9.9,-39.2,12,7.6)])
    # GVS two sockets, approximate envelope based on source photograph.
    ports=join([box(11,9,8,x,30.5,7.6) for x in (-10,3)])
    uart=box(10,13,2.5,19,8,8.0)
    hardware=[]
    for x,y in CASE_SCREWS:
        hardware.append(dict(id='M3_cover_screw',shape=cyl(3,16,x,y,9).union(cyl(5.5,3,x,y,25)),color='#a7abb2',role='hardware'))
        hardware.append(dict(id='M3_cover_nut',shape=hexprism(5.5,2.4,x,y,10.15).cut(cyl(3,2.6,x,y,10.05)),color='#a7abb2',role='hardware'))

    parts=[
        dict(id='E01_ceiling_base',label='Ceiling base with elevated rails',shape=base,
             color='#b8c8cc',qty=1,print_rotation=None,
             notes='Print ceiling face on bed; no supports. Two4.6mm ceiling holes, pitch100mm.'),
        dict(id='E02_removable_cover',label='One-piece removable downward cover',shape=cover,
             color='#587c8c',qty=1,print_rotation=((0,0,0),(1,0,0),180),
             notes='Print exterior closed floor on bed; cavity faces upward. No supports.'),
        dict(id='E03_ESP_retainer',label='ESP rail end stop',shape=retainers[0][1],
             color='#e4a757',qty=1,print_rotation=None,notes='M2.5x8 plus captured M2.5 nut.'),
        dict(id='E04_IMU_retainer',label='IMU rail end stop',shape=retainers[1][1],
             color='#e4a757',qty=1,print_rotation=None,notes='M2.5x8 plus captured M2.5 nut.'),
    ]
    electronics=[
        dict(id='URT2_PCB',shape=pcb_urt,color='#b94d4f',role='electronics'),
        dict(id='ESP_PCB',shape=pcb_esp,color='#22516a',role='electronics'),
        dict(id='IMU_PCB',shape=pcb_imu,color='#243532',role='electronics'),
        dict(id='ESP_USB',shape=usb,color='#bec6ca',role='electronics'),
        dict(id='ESP_chips',shape=esp_chips,color='#2c3035',role='electronics'),
        dict(id='IMU_chip',shape=imu_chip,color='#242b30',role='electronics'),
        dict(id='URT_components',shape=urt_components,color='#3a4f67',role='electronics'),
        dict(id='URT_GVS_ports',shape=ports,color='#f0ecdc',role='electronics'),
        dict(id='URT_UART_header',shape=uart,color='#303945',role='electronics'),
    ]
    return dict(parts=parts,electronics=electronics,hardware=hardware,
                dimensions=dict(body=[W,D,H],with_ears=[112,D,H],ceiling_hole_pitch=100,
                    pcb_underside_gap=PCB_Z-BASE,rail_lateral_clearance=.3,
                    rail_vertical_clearance=.4,urt_hole_centres=[48.5,28.7]),
                notes=[
                    'URT2 is rotated180deg: GVS at +Y, terminal access at-X, UART toward+X.',
                    'Assemble ESP first, then IMU: IMU occupies the ESP rail entry path.',
                    'Load the two M2.5 rail-stop nuts from the ceiling face before mounting base.',
                    'Four URT M2.5x6 screws and floating side-loaded M2.5 nuts.',
                    'Three M3x16 screws, standard M3 nuts, optional0.5mm washers close cover.',
                    'Electronic component models are approximate envelopes, not fit verification.'
                ])

def build_enclosure_fit_coupons():
    """Short rail sections: partially insert the actual PCB to check the fit."""
    result=[]
    for ident,width,thick,length,lip in [('C01_ESP_rail',21,1.6,14,3.4),
                                        ('C02_IMU_rail',15,1.0,10,1.2)]:
        shape=box(length+4,width+5,BASE).union(board_rails(0,0,length,width,thick,lip))
        result.append(dict(id=ident,label=ident,shape=shape,color='#e4a757',qty=1,
                    print_rotation=None,
                    notes='Fit coupon only: insert actual PCB partly from open end; verify edge solder clearance.'))
    return result

def print_shape(part):
    shape=part['shape']
    if part.get('print_rotation'):
        shape=shape.rotate(*part['print_rotation'])
    bb=shape.val().BoundingBox()
    return shape.translate((-(bb.xmin+bb.xmax)/2,-(bb.ymin+bb.ymax)/2,-bb.zmin))

if __name__=='__main__':
    import json
    data=build_enclosure()
    print(json.dumps({'dimensions':data['dimensions'],'parts':[
        {'id':p['id'],'valid':p['shape'].val().isValid(),
         'solids':len(p['shape'].solids().vals()),'volume_mm3':p['shape'].val().Volume()}
         for p in data['parts']]},indent=2))

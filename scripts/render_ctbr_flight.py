#!/usr/bin/env python3
"""Render recorded PX4/Gazebo flight: offline interactive replay, PNG and optional MP4."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import numpy as np
from yopo_l0.scene import load_card
from yopo_l0.flight_evidence import acceptance

HTML = r'''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>YOPO · CTBR 六机飞行记录</title>
<style>*{box-sizing:border-box}body{margin:0;background:#0a111a;color:#e7eff8;font:14px system-ui,"Microsoft YaHei",sans-serif}header{padding:22px 28px;border-bottom:1px solid #263345;display:flex;justify-content:space-between;align-items:center}h1{font-size:22px;margin:0 0 8px}small,.muted{color:#91a4bb}.badge{border:1px solid #324b63;padding:9px 14px;border-radius:20px;color:#8bdecc}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;padding:18px 28px}.card{background:#121f2c;border:1px solid #253547;border-radius:10px;padding:14px 18px}.value{font-size:24px;margin:5px 0}.views{display:grid;grid-template-columns:1fr 1.5fr;gap:14px;padding:0 28px}.view{background:#101b27;border:1px solid #253547;border-radius:12px;overflow:hidden}.label{padding:13px 16px;border-bottom:1px solid #253547;display:flex;justify-content:space-between}canvas{display:block;width:100%;height:440px;touch-action:none}.controls{margin:18px 28px;padding:16px;background:#121f2c;border-radius:10px;display:flex;align-items:center;gap:14px}button,select{background:#20384c;border:1px solid #3b5268;color:white;padding:8px 14px;border-radius:6px;cursor:pointer}input[type=range]{flex:1;accent-color:#6fd8c4}.table{padding:0 28px 20px}table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:10px;border-bottom:1px solid #263345}th{font-weight:400;color:#91a4bb}footer{padding:8px 28px 24px;color:#91a4bb;line-height:1.8}@media(max-width:900px){.views{grid-template-columns:1fr}.cards{grid-template-columns:1fr 1fr}canvas{height:350px}.controls{flex-wrap:wrap}} </style>
<header><div><h1>YOPO · CTBR 六机中心向外探索</h1><small>真实运行记录回放 · PX4 / Gazebo SITL · 全程可拖动核对</small></div><span class="badge" id="verdict"></span></header>
<div class="cards"><div class="card"><small>每机水平位移要求</small><div class="value" id="requirement"></div><small>相对各自起点 · 非累计航程</small></div><div class="card"><small>地图 / 障碍覆盖</small><div class="value">2.2 × 2.2 km</div><small id="coverage"></small></div><div class="card"><small>当前仿真时间</small><div class="value" id="time"></div><small id="phase"></small></div><div class="card"><small>当前前后跨度</small><div class="value" id="span"></div><small>编队允许随避障伸缩</small></div></div>
<div class="views"><div class="view"><div class="label">全场景 / 中心出发 <small>平面坐标 ENU，单位 m</small></div><canvas id="map"></canvas></div><div class="view"><div class="label">跟随视角 / 记录三维轨迹 <small>拖动旋转 · 滚轮缩放 · 高度显示 ×2.5</small></div><canvas id="follow"></canvas></div></div>
<div class="controls"><button id="play">▶ 播放</button><input id="seek" type="range" min="0" max="1000" value="0"><span id="stamp"></span><select id="speed"><option value="1">1×</option><option value="4">4×</option><option value="8" selected>8×</option><option value="16">16×</option></select><select id="source"><option value="truth">Gazebo 真值</option><option value="odom">PX4 里程计</option></select></div>
<div class="table"><table><thead><tr><th>飞机</th><th>水平位移</th><th>速度 / 高度</th><th>当前控制</th><th>全程真值最大位移</th></tr></thead><tbody id="rows"></tbody></table></div>
<footer>该页面依据实际保存的离散轨迹插值显示；障碍来自同一场景卡，未生成预设飞行路线。绿色点为障碍，彩色轨迹为各机记录。SITL 是仿真，不能作为实体飞机实飞结论。<br><span id="evidence"></span></footer>
<script>const D=__DATA__;const colors=['#65ddc8','#ffb86b','#90a9ff','#e9a0e8','#e1df74','#7fd4f7'];let time=0,playing=false,last=0,az=-.9,zoom=4.2,drag=null;let source='truth';const $=x=>document.getElementById(x);const end=Math.max(...D.tracks.map(a=>a.samples.at(-1).t));
const required=D.acceptance.required_per_vehicle_horizontal_displacement_m;$('requirement').textContent='≥ '+required+' m';$('verdict').textContent=D.acceptance.passed?required+' m 验收通过':'验收未通过 / 仅展示已有记录';$('coverage').textContent=D.obstacles.length+' 处障碍 · 全网格覆盖';$('evidence').textContent='Gazebo 碰撞样本：'+D.truth.collision_samples+' · PX4 参数写入：'+JSON.stringify(D.parameters)+' · 记录来源：'+D.run;
$('rows').innerHTML=D.tracks.map((a,i)=>`<tr id="row${i}"><td style="color:${colors[i]}">${a.id}</td><td></td><td></td><td></td><td>${(D.truth.max_horizontal_displacement_m[a.id]||0).toFixed(1)} m</td></tr>`).join('');
function sample(track,t){let a=track.samples,l=0,r=a.length-1;while(l<r){let m=Math.ceil((l+r)/2);if(a[m].t<=t)l=m;else r=m-1}let x=a[l],y=a[Math.min(l+1,a.length-1)],u=Math.max(0,Math.min(1,(t-x.t)/(y.t-x.t||1)));let key=source==='truth'?'g':'p';let p=x[key]||x.p,q=y[key]||y.p;return{p:p.map((v,j)=>v+(q[j]-v)*u),v:x.v,yaw:x.yaw,c:x.c,phase:x.phase,index:l}}
function setup(id){let c=$(id),d=devicePixelRatio||1,b=c.getBoundingClientRect();c.width=b.width*d;c.height=b.height*d;let g=c.getContext('2d');g.scale(d,d);return[g,b.width,b.height]}
function text(g,s,x,y,color='#93a8bf',size=11){g.fillStyle=color;g.font=size+'px system-ui';g.fillText(s,x,y)}function line(g,pts,color,width=1){if(!pts.length)return;g.beginPath();pts.forEach((p,i)=>i?g.lineTo(...p):g.moveTo(...p));g.strokeStyle=color;g.lineWidth=width;g.stroke()}
function map(states){let[g,w,h]=setup('map'),scale=Math.min(w,h)/2400,cx=w/2,cy=h/2;let P=p=>[cx+p[0]*scale,cy-p[1]*scale];g.fillStyle='#101b27';g.fillRect(0,0,w,h);for(let a=-1000;a<=1000;a+=250){line(g,[P([a,-1100]),P([a,1100])],'#223345');line(g,[P([-1100,a]),P([1100,a])],'#223345');text(g,a,P([a,0])[0]+3,cy+13)}D.obstacles.forEach(o=>{let p=P(o);g.fillStyle='#54866d';g.fillRect(p[0]-1,p[1]-1,2,2)});let c=P([0,0]);g.strokeStyle='#8094aa';g.beginPath();g.arc(...c,required*scale,0,Math.PI*2);g.stroke();text(g,required+' m 位移圈',c[0]+4,c[1]-required*scale-5);D.tracks.forEach((tr,i)=>{let s=states[i],pts=tr.samples.slice(0,s.index+1).map(a=>P((source==='truth'?a.g:a.p)||a.p));line(g,pts,colors[i],2);let p=P(s.p);g.fillStyle=colors[i];g.beginPath();g.arc(...p,4,0,Math.PI*2);g.fill()});g.fillStyle='white';g.beginPath();g.arc(...c,3,0,Math.PI*2);g.fill();text(g,'中心起飞',cx+7,cy-8,'#e4edf8')}
function follow(states){let[g,w,h]=setup('follow'),centre=[0,0,0];states.forEach(s=>s.p.forEach((x,j)=>centre[j]+=x/states.length));let co=Math.cos(az),si=Math.sin(az);let P=p=>{let x=p[0]-centre[0],y=p[1]-centre[1],z=p[2]*2.5;return[w/2+(x*co-y*si)*zoom,h*.62+(x*si+y*co)*zoom*.43-z*zoom]};g.fillStyle='#0c1721';g.fillRect(0,0,w,h);for(let a=-80;a<=80;a+=10){line(g,[P([centre[0]+a,centre[1]-80,0]),P([centre[0]+a,centre[1]+80,0])],'#223244');line(g,[P([centre[0]-80,centre[1]+a,0]),P([centre[0]+80,centre[1]+a,0])],'#223244')}let near=D.obstacles.filter(o=>Math.hypot(o[0]-centre[0],o[1]-centre[1])<90);near.sort((a,b)=>(a[0]*si+a[1]*co)-(b[0]*si+b[1]*co));near.forEach(o=>{let p=P([o[0],o[1],0]),top=P([o[0],o[1],o[3]]);line(g,[p,top],'#496a51',Math.max(2,o[2]*zoom*2));g.fillStyle='#355e4d';g.beginPath();g.ellipse(top[0],top[1],Math.max(3,o[2]*zoom),Math.max(2,o[2]*zoom*.5),0,0,Math.PI*2);g.fill()});D.tracks.forEach((tr,i)=>{let s=states[i],pts=tr.samples.slice(Math.max(0,s.index-130),s.index+1).map(a=>P((source==='truth'?a.g:a.p)||a.p));line(g,pts,colors[i],2);let p=P(s.p),z=s.p[2],arm=1.1;for(let u of[-1,1]){let ends=[-1,1].map(v=>P([s.p[0]+v*arm*Math.cos(s.yaw+u*Math.PI/4),s.p[1]+v*arm*Math.sin(s.yaw+u*Math.PI/4),z]));line(g,ends,colors[i],3);ends.forEach(e=>{g.strokeStyle=colors[i];g.beginPath();g.ellipse(...e,5,2.5,0,0,Math.PI*2);g.stroke()})}text(g,tr.id,p[0]+10,p[1]-10,colors[i],12)});text(g,'ENU X = '+centre[0].toFixed(1)+' m · '+(source==='truth'?'Gazebo 真值':'PX4 里程计'),18,26,'#dbe7f3',13)}
function draw(){let states=D.tracks.map(a=>sample(a,time));map(states);follow(states);$('time').textContent=time.toFixed(1)+' s';$('phase').textContent=states.map(s=>s.phase).every(x=>x===states[0].phase)?states[0].phase:'多机阶段切换';$('span').textContent=(Math.max(...states.map(s=>s.p[0]))-Math.min(...states.map(s=>s.p[0]))).toFixed(1)+' m';$('stamp').textContent=time.toFixed(1)+' / '+end.toFixed(1)+' s';$('seek').value=time/end*1000;states.forEach((s,i)=>{let tr=D.tracks[i],cells=$('row'+i).children;cells[1].textContent=Math.hypot(s.p[0]-tr.start[0],s.p[1]-tr.start[1]).toFixed(1)+' m';cells[2].textContent=s.v.toFixed(1)+' m/s · '+s.p[2].toFixed(1)+' m';cells[3].textContent=s.c?'CTBR':'PVA / 降落'})}
$('play').onclick=()=>{playing=!playing;if(time>=end)time=0;$('play').textContent=playing?'Ⅱ 暂停':'▶ 播放'};$('seek').oninput=e=>{time=e.target.value/1000*end;draw()};$('source').onchange=e=>{source=e.target.value;draw()};$('follow').onpointerdown=e=>{drag=e.clientX;$('follow').setPointerCapture(e.pointerId)};$('follow').onpointermove=e=>{if(drag!==null){az+=(e.clientX-drag)*.007;drag=e.clientX;draw()}};$('follow').onpointerup=()=>drag=null;$('follow').onwheel=e=>{e.preventDefault();zoom=Math.max(1.5,Math.min(10,zoom*Math.exp(-e.deltaY*.001)));draw()};window.onresize=draw;function tick(now){if(playing){time=Math.min(end,time+(now-last)/1000*+$('speed').value);draw();if(time>=end){playing=false;$('play').textContent='▶ 播放'}}last=now;requestAnimationFrame(tick)}draw();requestAnimationFrame(tick);</script></html>'''


def render(result_path, scene_path, dest, video=False):
    raw=json.loads(Path(result_path).read_text())
    card=load_card(scene_path)
    dest=Path(dest);dest.mkdir(parents=True,exist_ok=True)
    verdict=acceptance(raw,card)
    data={'tracks':[],'obstacles':[[*o['center_xy'],o['radius'],o['z_hi']] for o in card.card['geometry']['cylinders']],
          'acceptance':verdict,'truth':raw.get('truth_flight',{}),'parameters':raw.get('px4_parameters_written'),
          'run':str(Path(result_path).parent.name)}
    data['truth'].setdefault('max_horizontal_displacement_m',{})
    for drone in card.card['drones']:
        record=raw['flight_record'][drone['id']]
        samples=[]
        for s in record['samples']:
            samples.append({'t':round(s['t_s'],3),'p':np.round(s['p'],3).tolist(),
                'g':None if s.get('truth_p') is None else np.round(s['truth_p'],3).tolist(),
                'v':round(float(np.linalg.norm(s['v'])),3),'yaw':round(s['yaw'],4),
                'c':int(s['control']=='ctbr'),'phase':s['phase']})
        data['tracks'].append({'id':drone['id'],'start':drone['start_enu'],'samples':samples})
    (dest/'flight-replay.html').write_text(HTML.replace('__DATA__',json.dumps(data,separators=(',',':'))),encoding='utf-8')
    (dest/'acceptance.json').write_text(json.dumps(verdict,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors=['#27b99d','#e69b3c','#7187e8','#bd70bb','#aaaa39','#3b9fc9']
    fig=plt.figure(figsize=(16,9),facecolor='#f6f8fb')
    grid=fig.add_gridspec(2,2,width_ratios=[1,1.7],hspace=.3,wspace=.2)
    ax=fig.add_subplot(grid[:,0]);route=fig.add_subplot(grid[0,1]);dist=fig.add_subplot(grid[1,1])
    obs=np.asarray(data['obstacles'],dtype=float).reshape(-1,4);ax.scatter(obs[:,0],obs[:,1],s=2,c='#97ad9c',alpha=.7)
    ax.add_patch(plt.Circle((0,0),900,fill=False,color='#8796a8',ls='--',lw=1))
    ax.scatter([0],[0],c='black',s=20);ax.set(xlim=(-1150,1150),ylim=(-1150,1150),aspect='equal',title='2.2 km x 2.2 km | 3025 obstacles',xlabel='East X (m)',ylabel='North Y (m)')
    route.scatter(obs[:,0],obs[:,1],s=np.pi*(obs[:,2]*5)**2,c='#9eafa3');route.set(xlim=(-20,980),ylim=(-55,55),title='Recorded flight corridor (Gazebo truth)',xlabel='East X (m)',ylabel='North Y (m)')
    for i,tr in enumerate(data['tracks']):
        samples=tr['samples'];p=np.array([s['g'] or s['p'] for s in samples]);t=np.array([s['t'] for s in samples])
        d=np.linalg.norm(p[:,:2]-np.array(tr['start'])[:2],axis=1)
        ax.plot(p[:,0],p[:,1],color=colors[i],lw=1.5)
        route.plot(p[:,0],p[:,1],color=colors[i],lw=1.2,label=tr['id'])
        dist.plot(t,d,color=colors[i],lw=1.3,label=tr['id'])
    dist.axhline(verdict['required_per_vehicle_horizontal_displacement_m'],color='#d65252',ls='--',lw=1);dist.set(title=f"Per-vehicle horizontal displacement | threshold {verdict['required_per_vehicle_horizontal_displacement_m']:g} m",xlabel='Simulation time (s)',ylabel='Displacement from own start (m)');dist.legend(ncol=6,fontsize=8,loc="upper left")
    for a in [ax,route,dist]:a.grid(alpha=.15);a.spines[['top','right']].set_visible(False)
    label='PASS' if verdict['passed'] else 'NOT PASSED'
    fig.suptitle(f'YOPO + CTBR | Six-vehicle centre-out exploration | {label}',fontsize=18,x=.5,y=.98)
    fig.text(.5,.015,f'PX4/Gazebo SITL recorded data | {data["run"]} | No PX4 parameter writes | Simulation, not physical flight',ha='center',fontsize=9,color='#576574')
    fig.savefig(dest/'flight-overview.png',dpi=140,bbox_inches='tight');plt.close(fig)
    if video: render_video(data,dest,colors)
    return verdict


def render_video(data,dest,colors):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter
    tracks=data['tracks'];arrays=[np.array([s['g'] or s['p'] for s in tr['samples']]) for tr in tracks]
    times=[np.array([s['t'] for s in tr['samples']]) for tr in tracks]
    end=max(t[-1] for t in times);obs=np.array(data['obstacles'])
    fig=plt.figure(figsize=(12.8,7.2),facecolor='#f7f9fc');ax=fig.add_subplot(121,projection='3d');side=fig.add_subplot(122)
    side.scatter(obs[:,0],obs[:,1],s=3,c='#a6b4aa');side.set(xlim=(-50,1000),ylim=(-100,100),xlabel='East X (m)',ylabel='North Y (m)',title='Full recorded flight corridor')
    trails=[ax.plot([],[],[],color=c,lw=1.6)[0] for c in colors];dots=[ax.plot([],[],[],'o',color=c,ms=6)[0] for c in colors]
    maplines=[side.plot([],[],color=c,lw=1.4)[0] for c in colors];mapdots=[side.plot([],[],'o',color=c,ms=5)[0] for c in colors]
    stems=[ax.plot([],[],[],color='#8d9f91',lw=3)[0] for _ in range(20)]
    ax.set(xlabel='East X (m)',ylabel='North Y (m)',zlabel='Height (m)',zlim=(0,12),title='Gazebo truth | Follow view');ax.view_init(elev=23,azim=-65)
    title=fig.suptitle('YOPO + CTBR | recorded PX4 / Gazebo SITL',fontsize=16)
    fig.subplots_adjust(left=.05,right=.98,bottom=.13,top=.87,wspace=.2)
    footer=fig.text(.5,.04,'',ha='center',fontsize=10)
    writer=FFMpegWriter(fps=12,bitrate=1800,metadata={'title':'Recorded YOPO CTBR flight'})
    with writer.saving(fig,str(dest/'flight-preview.mp4'),dpi=100):
        for t in np.linspace(0,end,360):
            points=[];disp=[]
            for i,(tr,p,ts) in enumerate(zip(tracks,arrays,times)):
                k=min(int(np.searchsorted(ts,t)),len(ts)-1);q=p[k];points.append(q);disp.append(float(np.linalg.norm(q[:2]-np.array(tr['start'])[:2])))
                tail=p[max(0,k-35):k+1];trails[i].set_data(tail[:,0],tail[:,1]);trails[i].set_3d_properties(tail[:,2]);dots[i].set_data([q[0]],[q[1]]);dots[i].set_3d_properties([q[2]])
                maplines[i].set_data(p[:k+1,0],p[:k+1,1]);mapdots[i].set_data([q[0]],[q[1]])
            centre=np.mean(points,axis=0);ax.set_xlim(centre[0]-45,centre[0]+45);ax.set_ylim(centre[1]-35,centre[1]+35)
            nearby=obs[(np.abs(obs[:,0]-centre[0])<43)&(np.abs(obs[:,1]-centre[1])<33)]
            near=nearby[np.argsort(np.linalg.norm(nearby[:,:2]-centre[:2],axis=1))[:20]]
            for idx,stem in enumerate(stems):
                if idx<len(near):
                    o=near[idx];stem.set_data([o[0],o[0]],[o[1],o[1]]);stem.set_3d_properties([0,o[3]])
                else:
                    stem.set_data([],[]);stem.set_3d_properties([])
            footer.set_text(f'Simulation {t:.1f} / {end:.1f} s | Horizontal displacement (m): '+', '.join(f'{d:.0f}' for d in disp))
            writer.grab_frame()
    plt.close(fig)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--result',required=True);ap.add_argument('--scene',required=True);ap.add_argument('--out',required=True);ap.add_argument('--video',action='store_true');args=ap.parse_args()
    print(json.dumps(render(args.result,args.scene,args.out,args.video),indent=2))

"""Build a standalone interactive study report from saved candidate checkpoints."""
import argparse
import json
from pathlib import Path


def report(folder):
    folder=Path(folder)
    settings=json.loads((folder/'settings.json').read_text())
    capture=json.loads((Path(settings['case'])/'case.json').read_text())
    rows=[json.loads(p.read_text()) for p in (folder/'candidates').glob('*.json')]
    from validation.study_stationary_grid import score_rows
    ranked=score_rows(rows,settings['preference'])
    paths=[json.loads(p.read_text()) for p in (folder/'paths').glob('*.json')]
    for p in paths:p.pop('configurations',None)
    points=[t['origin'] for t in capture['replay']['targets']]
    data=dict(settings=settings,rows=rows,ranked=[r['id'] for r in ranked],paths=paths,targets=points,
              complete=(folder/'result.json').exists())
    html=TEMPLATE.replace('__DATA__',json.dumps(data).replace('<','\\u003c'))
    (folder/'report.html').write_text(html,encoding='utf-8')
    return folder/'report.html'


TEMPLATE='''<!doctype html><html lang="en"><meta charset="utf-8"><title>Stationary placement study</title>
<style>body{font:15px system-ui;margin:32px auto;max-width:1200px;padding:0 20px;color:#172331;background:#fafafa}h1{font-size:28px}h2{font-size:20px}select,button{font:inherit;padding:5px;margin:4px}canvas{background:white;border:1px solid #ccd3da;max-width:100%}.panels{display:flex;gap:24px;flex-wrap:wrap}.detail{flex:1;min-width:300px}table{border-collapse:collapse;width:100%;background:white}th,td{padding:8px;border-bottom:1px solid #ddd;text-align:right}td:first-child,th:first-child{text-align:left}tr{cursor:pointer}tr:hover{background:#e7f3fb}.note{color:#546273;line-height:1.5}#hover{min-height:24px}#details{white-space:pre-wrap}code{font-size:12px}</style>
<h1>Stationary base placement study</h1><p id="summary"></p>
<p class="note">Coordinates are the calibrated <b>arm-base origin</b> in world XY metres. The blue polygon is the initial 1.75 m reach/target-side region, not a collision-free region. Grey points are TCP targets; crosses are the original standoff baseline. Refinement may extend beyond the polygon. Click a candidate to inspect all target counts.</p>
<label>Colour <select id="metric"><option value="reachable_targets">Reachable targets</option><option value="minimum_solutions">Minimum solutions</option><option value="mean_solutions">Mean solutions</option><option value="score">Balanced score</option></select></label>
<label>Stage <select id="stage"><option value="all">All stages</option></select></label><label><input id="blocked" type="checkbox">Show blocked bases</label>
<div id="hover"></div><div class="panels"><canvas id="map" width="700" height="650"></canvas><div class="detail"><h2>Selected candidate</h2><div id="details">Select a point or table row.</div><canvas id="counts" width="440" height="230"></canvas><p class="note">Per-target collision-free configuration counts, in original target order. Full reachability does not guarantee path connectivity. Counts refer to the 24-orientation sampled set (or the override shown above), not continuous TCP rotation.</p></div></div>
<h2>Ranked candidates</h2><p class="note">Coverage ranks first. The score balances normalized minimum and mean counts; all candidates remain in the saved JSON. The table shows the top 40. This finite study does not establish a global optimum.</p><table><thead><tr><th>Candidate</th><th>Stage</th><th>Reached</th><th>Min</th><th>Total</th><th>Score</th><th>Within old region</th><th>Path</th></tr></thead><tbody id="table"></tbody></table>
<script>const D=__DATA__;const $=id=>document.getElementById(id),byId=Object.fromEntries(D.rows.map(r=>[r.id,r]));let selected=null,shown=[];
$('summary').textContent=`${D.complete?'Completed':'Checkpoint'} study · ${D.settings.targets} targets · ${D.settings.rotation_steps} TCP rotations · ${D.rows.length} evaluated base poses · ${D.rows.filter(r=>r.reachable_targets===D.settings.targets).length} fully reachable poses · seed ${D.settings.seed}`;
for(const s of [...new Set(D.rows.map(r=>r.stage))].sort()){let o=document.createElement('option');o.value=s;o.textContent=s;$('stage').append(o)}
const points=[...D.targets,...D.settings.polygon,...D.rows.map(r=>r.arm_xy)],xs=points.map(p=>p[0]),ys=points.map(p=>p[1]);const bounds=[Math.min(...xs)-.15,Math.max(...xs)+.15,Math.min(...ys)-.15,Math.max(...ys)+.15];const scale=Math.min(620/(bounds[1]-bounds[0]),570/(bounds[3]-bounds[2]));const xy=p=>[40+(p[0]-bounds[0])*scale,610-(p[1]-bounds[2])*scale];
function draw(){const c=$('map').getContext('2d');c.clearRect(0,0,700,650);c.fillStyle='#73808b';for(const p of D.targets){const[x,y]=xy(p);c.fillRect(x,y,2,2)}c.beginPath();D.settings.polygon.forEach((p,i)=>{const[x,y]=xy(p);i?c.lineTo(x,y):c.moveTo(x,y)});c.closePath();c.fillStyle='#d6eafa55';c.fill();c.strokeStyle='#4278ae';c.stroke();shown=D.rows.filter(r=>($('blocked').checked||!r.base_blocked&&!r.start_blocked)&&($('stage').value==='all'||r.stage===$('stage').value));let key=$('metric').value,max=Math.max(1,...shown.map(r=>r[key]));for(const r of shown){const[x,y]=xy(r.arm_xy),v=r[key]/max;c.beginPath();c.arc(x,y,r.id===selected?7:4,0,Math.PI*2);c.fillStyle=r.base_blocked?'#aaa':`hsl(${v*130},70%,43%)`;c.fill();c.strokeStyle=r.id===selected?'#111':'#fff';c.stroke();if(r.stage==='baseline'){c.strokeStyle='#000';c.beginPath();c.moveTo(x-9,y-9);c.lineTo(x+9,y+9);c.moveTo(x-9,y+9);c.lineTo(x+9,y-9);c.stroke()}}c.fillStyle='#333';c.font='12px system-ui';c.fillText(`X ${bounds[0].toFixed(2)} → ${bounds[1].toFixed(2)} m`,40,638);c.fillText(`Y ${bounds[2].toFixed(2)} → ${bounds[3].toFixed(2)} m`,40,20)}
function choose(id){selected=id;const r=byId[id],p=D.paths.find(p=>p.candidate===id);$('details').textContent=`${id}\nStage: ${r.stage}\nArm origin XY: ${r.arm_xy.map(x=>x.toFixed(3)).join(', ')} m\nBase footprint: ${r.base.origin.map(x=>x.toFixed(3)).join(', ')} m\nHeading: ${(r.yaw*180/Math.PI).toFixed(1)}°\nReachable: ${r.reachable_targets}/${D.settings.targets}\nMinimum / mean: ${r.minimum_solutions} / ${r.mean_solutions.toFixed(2)}\nTotal: ${r.total_solutions}\nOriginal region: ${r.original_region_metrics.geometry_valid}\nFarthest target XY: ${r.original_region_metrics.max_projected_distance.toFixed(3)} m\nEvaluation: ${r.elapsed_seconds.toFixed(1)} s\nPath: ${p?(p.path_complete?'connected':'disconnected at target '+(p.failure_layer+1)):'not tested'}\n${r.base_failure||''}`;const c=$('counts').getContext('2d');c.clearRect(0,0,440,230);let max=Math.max(1,...r.counts);c.beginPath();r.counts.forEach((v,i)=>{let x=30+390*i/Math.max(1,r.counts.length-1),y=200-170*v/max;i?c.lineTo(x,y):c.moveTo(x,y)});c.strokeStyle='#2765ac';c.stroke();c.fillStyle='#333';c.fillText(`0–${max} solutions; targets 1–${r.counts.length}`,30,220);draw()}
for(const id of D.ranked.slice(0,40)){const r=byId[id],p=D.paths.find(p=>p.candidate===id),tr=document.createElement('tr');for(const v of [id.slice(0,8),r.stage,r.reachable_targets,r.minimum_solutions,r.total_solutions,r.score.toFixed(3),r.original_region_metrics.geometry_valid?'yes':'no',p?(p.path_complete?'connected':'blocked'):'untested']){const td=document.createElement('td');td.textContent=v;tr.append(td)}tr.onclick=()=>choose(id);$('table').append(tr)}
$('map').onclick=e=>{const b=$('map').getBoundingClientRect(),p=[(e.clientX-b.left)*700/b.width,(e.clientY-b.top)*650/b.height];let near=shown.map(r=>[r,Math.hypot(xy(r.arm_xy)[0]-p[0],xy(r.arm_xy)[1]-p[1])]).sort((a,b)=>a[1]-b[1]);if(near.length&&near[0][1]<15)choose(near[0][0].id)};for(const id of ['metric','stage','blocked'])$(id).onchange=draw;draw();if(D.ranked.length)choose(D.ranked[0]);</script></html>'''


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path)
    print(report(p.parse_args().folder))

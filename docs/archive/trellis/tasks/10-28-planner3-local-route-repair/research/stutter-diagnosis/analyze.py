import json,sys,math,statistics as st
r=json.load(open(sys.argv[1]));rows=[x['data'] for x in r['transactions'] if len(x['data'])>=33 and x['data'][21]==0]
def stats(a):
 a=sorted(x for x in a if math.isfinite(x))
 return {k:round(v,2) for k,v in {'n':len(a),'mean':st.mean(a),'p50':st.median(a),'p95':a[int((len(a)-1)*.95)],'max':max(a)}.items()} if a else {}
for i,label in [(4,'entry gap'),(11,'observation callback lock wait'),(12,'planner lock wait'),(13,'FIRI'),(14,'global A*'),(16,'MPPI'),(17,'authorization'),(18,'critical cycle'),(19,'total cycle'),(22,'A* hard checks'),(23,'A* soft checks'),(24,'A* grid setup'),(32,'LFPC integrate')]:print(label,stats([x[i]*1000 for x in rows]))
print('cycle details: entryGap / A* / FIRI / MPPI / critical / total ms')
for x in rows[:20]:print(' '.join(f'{x[i]*1000:.1f}' for i in [4,14,13,16,18,19]))
moves=[]
for x in r['odom']:
 if x['t']<r['goal_time']:continue
 if not moves or math.hypot(x['x']-moves[-1]['x'],x['y']-moves[-1]['y'])>.001:moves.append(x)
gaps=[(b['t']-a['t'])*1000 for a,b in zip(moves,moves[1:])];bad=[x for x in gaps if x>150]
print('moving odom gap ms',stats(gaps),'gaps >150ms:',len(bad))
print('STOP reasons',sorted(set(x['text'] for x in r['stops'])))
print('final distance',min(math.hypot(x['x']-4.139,x['y']+4.681) for x in r['odom']))
if '--check' in sys.argv:
 assert rows and moves,'No active samples'
 assert len(bad)==0,f'STUTTER: {len(bad)} moving updates exceed150ms, max {max(gaps):.1f}ms'

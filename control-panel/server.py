#!/usr/bin/env python3
"""Local USB board-test console. Binds to loopback only; no cloud dependency."""
import csv,datetime,io,json,pathlib,re,secrets,sys,threading,time,atexit,math
from http.server import ThreadingHTTPServer,BaseHTTPRequestHandler
from urllib.parse import urlparse
import serial
from serial.tools import list_ports
ROOT=pathlib.Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'firmware-app/host'))
from glove import parse_capture, capture_metadata
TOKEN=secrets.token_urlsafe(32)
PORT=8775
class Hub:
 def __init__(self):
  self.lock=threading.RLock();self.link=None;self.port='';self.status={};self.events=[];self.last_seen=time.monotonic();self.last_capture=None;self.readings={};self.last_poll=0;self.diagnostics={};self.last_coil=0;self.pose_calibration=None;self.motion_calibration=None;self.glove_calibrations={};self.pose_stabilizers={}
 def log(self,message,level='info'):
  self.events.append({'time':datetime.datetime.now().strftime('%H:%M:%S'),'message':message,'level':level});self.events=self.events[-150:]
 def ports(self):
  return [{'port':p.device,'name':p.description,'serial':p.serial_number} for p in list_ports.comports() if p.vid==0x0483 and p.pid==0x5740]
 def state(self):
  return {'connected':self.link is not None,'port':self.port,'status':self.status.copy(),'events':list(self.events),'readings':self.readings.copy(),'capture':self.last_capture,'diagnostics':self.diagnostics.copy()}
 def exchange(self,command):
  if not self.link:raise ValueError('Connect the hand hub first.')
  self.link.write((command+'\n').encode());self.link.flush()
  line=self.link.readline()
  if not line:raise OSError('The hub did not respond. Reconnect USB and try Connect again.')
  return line
 def refresh(self):
  line=self.exchange('status').decode(errors='replace').strip()
  m=re.fullmatch(r'OK firmware=(\S+) core=(\d+) sensors=([01]) found=(\d+) coil_drive_build=([01])',line)
  if not m or (m[1],m[5]) not in [('0.1.0-bench','0'),('0.2.0-diagnostics','1'),('0.2.1-diagnostics','1'),('0.2.2-diagnostics','1'),('0.2.3-diagnostics','1'),('0.2.4-diagnostics','1'),('0.2.5-diagnostics','1'),('0.2.6-diagnostics','1'),('0.2.7-diagnostics','1')]:raise OSError('Unsupported firmware identity or coil capability. Expected bench 0.1 or diagnostics 0.2.')
  self.status={'firmware':m[1],'core':int(m[2]),'sensors':bool(int(m[3])),'found':int(m[4]),'coil_drive':m[5]=='1'}
  if not self.status['sensors']:self.readings={}
 def close(self):
  if self.link:
   try:self.link.write(b'sensors off\n');self.link.flush();self.link.dtr=False
   except Exception:pass
   try:self.link.close()
   except Exception:pass
  self.link=None;self.status={};self.readings={};self.port='';self.last_capture=None;self.diagnostics={};self.pose_calibration=None;self.motion_calibration=None;self.glove_calibrations={};self.pose_stabilizers={}
 def emergency(self):
  # One-byte out-of-band firmware stop bypasses the transaction lock.
  if self.link:
   self.link.write(b'!')
   return {'requested':True,'message':'Emergency stop sent; sensor-off confirmation follows.'}
  return {'requested':False,'message':'No USB connection. Unplug hub if needed.'}
 def action(self,action,data):
  with self.lock:
   self.last_seen=time.monotonic()
   try:
    if action=='connect':
     if self.link: self.refresh();return self.state()
     ports=self.ports();port=data.get('port')
     if not port and len(ports)==1:port=ports[0]['port']
     if port not in [p['port'] for p in ports]:raise ValueError('Select a detected glove USB port. If none appear, reconnect the hub without holding BOOT.')
     self.link=serial.Serial(port,115200,timeout=3,write_timeout=3,exclusive=True);self.port=port
     self.link.dtr=True;time.sleep(.3);self.link.reset_input_buffer();self.refresh();self.log('Hub connected · '+self.status['firmware'])
    elif action=='disconnect':self.close();self.log('Disconnected. Sensor power off requested.')
    elif action=='status':self.refresh()
    elif action in ('power_on','power_off','probe'):
     if action=='probe' and not self.status.get('sensors'):raise ValueError('Turn sensor power on before checking fingertips.')
     command={'power_on':'sensors on','power_off':'sensors off','probe':'probe'}[action]
     reply=self.exchange(command).decode(errors='replace').strip();self.log(reply,'error' if reply.startswith('ERR') else 'info');self.refresh()
     if reply.startswith('ERR'):raise ValueError(reply)
    elif action=='read':
     index=int(data.get('sensor',0))
     if index not in range(1,6) or not self.status.get('found',0)&(1<<(index-1)):raise ValueError('Check fingertips first, then select a detected port.')
     reply=self.exchange('read '+str(index)).decode(errors='replace').strip()
     m=re.fullmatch(r'ADC (\d) status=([0-9a-f]+) ch=(-?\d+),(-?\d+),(-?\d+),(-?\d+)',reply)
     if not m:raise ValueError(reply)
     self.readings[str(index)]={'channels':[int(m[i]) for i in range(3,7)],'status':m[2],'time':datetime.datetime.now().strftime('%H:%M:%S')}
    elif action in ('currents','imu_probe','imu_on','imu_read','imu_off','coil_test'):
     if not self.status.get('coil_drive'):raise ValueError('Flash diagnostics firmware 0.2 first. Bench firmware cannot run this test.')
     if not self.status.get('sensors'):raise ValueError('Turn sensor power on first.')
     if action=='coil_test':
      axes=int(data.get('axes',0));ms=int(data.get('ms',10))
      if not 1<=axes<=7 or not 1<=ms<=100:raise ValueError('Select X/Y/Z and a 1–100 ms burst.')
      if data.get('bench_ready') is not True:raise ValueError('Confirm wiring and bench readiness before driving coils.')
      if time.monotonic()-self.last_coil<2:raise ValueError('Wait two seconds between coil tests.')
      self.last_coil=time.monotonic();command=f'coil_test {axes} {ms}'
     else:command=action.replace('_',' ')
     reply=self.exchange(command).decode(errors='replace').strip()
     if reply.startswith('ERR'):raise ValueError(reply)
     self.diagnostics[action]={'reply':reply,'time':datetime.datetime.now().strftime('%H:%M:%S')}
     if action=='coil_test':
      m=re.fullmatch(r'COIL result=(\S+) axes=(\d+) ms=(\d+) samples=(\d+) vdda_mv=(\d+) zero=(\d+),(\d+),(\d+) min=(\d+),(\d+),(\d+) max=(\d+),(\d+),(\d+)',reply)
      if not m:raise OSError('Malformed coil result; disconnecting.')
      if m[1]!='ok':raise ValueError('Coil test stopped: '+m[1])
      values=[int(m[i]) for i in range(2,15)];vdda=values[3];zero=values[4:7];lo=values[7:10];hi=values[10:13]
      self.diagnostics[action]['peak_ma']=[round(max(abs(lo[i]-zero[i]),abs(hi[i]-zero[i]))*vdda/4095/4,2) for i in range(3)]
      self.diagnostics[action]['note']='Sampled current only; verify peaks and frequency with scope. Near-zero current may mean an open coil.'
     self.log(reply)
    elif action=='axis_fit':
     from unified_tracking import read_capture,coherent_pose
     from axis_registration import solve_axis_moves,update_reference
     port=str(int(data.get('sensor',0)));groups=data.get('groups',{})
     if port not in getattr(self,'unified_profiles',{}):raise ValueError('This receiver needs its own movement fit first')
     if not isinstance(groups,dict) or set(groups)!=set(('start','x','y','z')):raise ValueError('Incomplete guided captures')
     profile=self.unified_profiles[port];parsed={};seen=set();previous=self.hand_reference['branch_seeds'].get(port)
     for name in ('start','x','y','z'):
      files=groups[name]
      if not isinstance(files,list) or len(files)!=4:raise ValueError('Four samples required per pose')
      parsed[name]=[]
      for file in files:
       if file in seen:raise ValueError('Duplicate capture')
       seen.add(file);values=read_capture(ROOT,file,int(port),self.port)
       pose=coherent_pose(profile['cal'],values,[[1,0,0],[0,1,0],[0,0,1]],previous)
       parsed[name].append(pose)
     self.exchange('sensors off');self.refresh()
     fit=solve_axis_moves(parsed);fit['sensor']=port;fit['files']=groups
     updated=update_reference(self.hand_reference,fit)
     file='hand-reference-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json'
     (ROOT/'calibrations'/file).write_text(json.dumps(updated,indent=2))
     if json.loads((ROOT/'calibrations'/file).read_text())!=updated:raise OSError('Reference readback failed')
     self.action('hand_load',{});self.diagnostics['axis_fit']=fit
    elif action=='hand_load':
     self.action('unified_load',{})
     refs=sorted((ROOT/'calibrations').glob('hand-reference-*.json'),reverse=True)
     if not refs:raise ValueError('No flat hand reference is saved')
     changed=getattr(self,'hand_reference_file',None)!=refs[0].name
     self.hand_reference_file=refs[0].name
     self.hand_reference=json.loads(refs[0].read_text())
     if self.hand_reference.get('device')!=self.port:raise ValueError('Hand reference belongs to another hub')
     self.hand_stabilizers={} if changed else getattr(self,'hand_stabilizers',{})
     for port,seed in self.hand_reference['branch_seeds'].items():
      profile=self.unified_profiles.get(port)
      if profile and profile['file']==self.hand_reference['profile_files'].get(port):
       key=(port,profile['file'])
       if changed:self.unified_previous[key]=seed
       else:self.unified_previous.setdefault(key,seed)
     self.diagnostics['hand_reference']=self.hand_reference
     self.diagnostics['hand_display_scale']=self.unified_profiles.get('1',{}).get('cal',{}).get('metres_per_unit')
     self.diagnostics['glove_profiles']=self.diagnostics['unified_profiles']
    elif action=='hand_flip':
     port=str(int(data.get('sensor',0)));profile=self.unified_profiles.get(port)
     model=port if profile else ('4' if '4' in self.unified_profiles else next(iter(self.unified_profiles)))
     key=(port,self.unified_profiles[model]['file']);previous=self.unified_previous.get(key)
     if previous is None:raise ValueError('Wait for a valid fingertip reading')
     self.unified_previous[key]=[-v for v in previous]
     if key in self.hand_stabilizers:self.hand_stabilizers[key].flip()
    elif action=='hand_frame':
     from hand_registration import apply_registration
     from pose_stabilizer import PoseStabilizer
     self.action('unified_frame',data);poses={}
     reference_scale=self.unified_profiles.get('1',{}).get('cal',{}).get('metres_per_unit')
     for port,value in self.diagnostics['unified_poses'].items():
      try:
       if 'error' in value:raise ValueError(value['error'])
       p=apply_registration(self.hand_reference,port,value['profile'],value,reference_scale)
       p=self.hand_stabilizers.setdefault((port,value['profile']),PoseStabilizer()).apply(p,time.monotonic())
       p['provisional']=p['shared_model'] or not p['metric'] or not p['mount_registered'];poses[port]=p
      except ValueError as e:poses[port]={'error':str(e),'provisional':True}
     self.diagnostics['glove_poses']=poses;self.diagnostics['glove_solved_capture']=self.last_capture['file']
     result=self.state();result['events']=[];return result
    elif action=='unified_load':
     from unified_tracking import load_profiles
     self.unified_profiles,errors=load_profiles(ROOT,self.port)
     self.unified_previous=getattr(self,'unified_previous',{})
     self.unified_stabilizers=getattr(self,'unified_stabilizers',{})
     master=self.unified_profiles.get('1') or next(iter(self.unified_profiles.values()),None)
     self.unified_source=(master.get('registration') or {}).get('rotation',[[1,0,0],[0,1,0],[0,0,1]]) if master else [[1,0,0],[0,1,0],[0,0,1]]
     self.diagnostics['unified_profiles']={k:{'file':v['file'],'units':v['cal']['units'],'sample_count':v['cal']['sample_count'],'held_out_rms':v['cal']['held_out_rms']} for k,v in self.unified_profiles.items()}
     self.diagnostics['unified_source']={'rotation':self.unified_source,'physical_alignment':'approximate saved J1 references' if master and master.get('registration') else 'electrical axes A/B/C; physical directions unverified'}
     self.diagnostics['unified_errors']=errors
    elif action=='unified_fit':
     from unified_tracking import fit_receivers
     self.exchange('sensors off');self.refresh()
     results=fit_receivers(ROOT,data.get('records'),self.port)
     self.diagnostics['unified_fit']=results
     self.action('unified_load',{})
    elif action=='unified_frame':
     from unified_tracking import coherent_pose
     self.action('capture_stream',data)
     profiles=getattr(self,'unified_profiles',{});poses={}
     for port,sp in self.last_capture['spectrum'].items():
      model=port if port in profiles else ('4' if '4' in profiles else next(iter(profiles),None))
      try:
       if not model:raise ValueError('Record movement to learn this receiver')
       if sp['clipped']:raise ValueError('Signal clipped')
       profile=profiles[model];key=(port,profile['file']);prior=self.unified_previous.get(key)
       p=coherent_pose(profile['cal'],[t.get('xyz_source_counts') for t in sp['amplitudes']],self.unified_source,prior)
       self.unified_previous[key]=p['position']
       from pose_stabilizer import PoseStabilizer
       p['hub_relative_position_m']=p['source_position'];p['display_rotation']=p['source_rotation']
       p=self.unified_stabilizers.setdefault(key,PoseStabilizer()).apply(p,time.monotonic())
       p['source_position']=p.pop('hub_relative_position_m');p['source_rotation']=p.pop('display_rotation')
       p.update(model_port=model,shared_model=model!=port,profile=profile['file'],capture=self.last_capture['file'])
       poses[port]=p
      except ValueError as e:poses[port]={'error':str(e),'model_port':model,'shared_model':model!=port}
     self.diagnostics['unified_poses']=poses
     result=self.state();result['events']=[];return result
    elif action=='tracking_frame':
     self.action('capture_stream',data)
     self.action('glove_solve',{})
     if self.status.get('firmware')!='0.2.7-diagnostics':self.action('imu_read',{})
     result=self.state();result['events']=[]
     return result
    elif action=='capture_stream':
     from stream_packet import parse_stream
     mask=int(data.get('mask',0))
     if self.status.get('firmware') not in ('0.2.6-diagnostics','0.2.7-diagnostics'):raise ValueError('Streaming needs firmware 0.2.6.')
     if data.get('bench_ready') is not True or not 1<=mask<=31 or mask&~self.status.get('found',0):raise ValueError('Invalid streaming receiver mask or bench readiness.')
     # Honor the interval by waiting; scheduler jitter must not kill the session.
     remaining=.015-(time.monotonic()-self.last_coil)
     if remaining>0:time.sleep(remaining)
     self.last_coil=time.monotonic()
     header=self.exchange(f'capture_stream {mask}')
     if not header.startswith(b'TRACK'):raise ValueError(header.decode(errors='replace').strip())
     try:sp,ticks,imu=parse_stream(self.link,header,mask)
     except ValueError as e:raise OSError(str(e)) from e
     if imu:self.diagnostics['imu_read']={'reply':imu,'time':datetime.datetime.now().strftime('%H:%M:%S')}
     self.last_capture={'file':'stream-'+str(time.monotonic_ns()),'mask':mask,'samples':128,'rate':24000,'time':datetime.datetime.now().strftime('%H:%M:%S'),'transmitter':True,'axes':7,'firmware':self.status['firmware'],'device_port':self.port,'phase_metadata':{'phase_ticks':ticks},'spectrum':sp,'raw_recording':False}
    elif action in ('capture','capture_tx','capture_live','capture_track'):
     mask=int(data.get('mask',0))
     if not 1<=mask<=31 or mask&~self.status.get('found',0):raise ValueError('Select only detected fingertip ports for capture.')
     drive=action in ('capture_tx','capture_live','capture_track');fast=action=='capture_live';tracking=action=='capture_track'
     if tracking and self.status.get('firmware') not in ('0.2.5-diagnostics','0.2.6-diagnostics','0.2.7-diagnostics'):raise ValueError('Flash firmware 0.2.5 for faster all-axis tracking.')
     if fast and (self.status.get('firmware') not in ('0.2.3-diagnostics','0.2.4-diagnostics','0.2.5-diagnostics','0.2.6-diagnostics','0.2.7-diagnostics') or mask&(mask-1)):raise ValueError('Live mode requires firmware 0.2.3 and one receiver.')
     if drive:
      axes=int(data.get('axes',0))
      if tracking and axes!=7:raise ValueError('Tracking mode requires all three axes.')
      if fast and axes!=4:raise ValueError('Live mode is Z only.')
      if not self.status.get('coil_drive'):raise ValueError('Flash diagnostics firmware 0.2 first.')
      if not 1<=axes<=7 or data.get('bench_ready') is not True:raise ValueError('Select coil axes and confirm bench readiness.')
      if time.monotonic()-self.last_coil<(0.32 if tracking else 0.1 if fast else 2):raise ValueError('Wait for the coil cooldown.')
      self.last_coil=time.monotonic()
     header=self.exchange(f'capture_track {mask}' if tracking else f'capture_live {mask}' if fast else f'capture_tx {mask} {axes}' if drive else 'capture '+str(mask))
     if not header.startswith(b'CAPTURE'):raise ValueError(header.decode(errors='replace').strip())
     try:rows=parse_capture(self.link,header)
     except ValueError as e:raise OSError(str(e)) from e
     name='capture-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.csv'
     folder=ROOT/'captures';folder.mkdir(exist_ok=True)
     with (folder/name).open('w',newline='') as f:
      writer=csv.writer(f);writer.writerow(['sensor','sample','status','x_raw','y_raw','z_raw','aux_raw']);writer.writerows(rows)
     self.last_capture={'file':name,'mask':mask,'samples':256,'rate':24000,'rows':rows,'time':datetime.datetime.now().strftime('%H:%M:%S'),'transmitter':drive,'axes':axes if drive else 0,'firmware':self.status.get('firmware'),'device_port':self.port,'phase_metadata':capture_metadata(header),'spectrum':spectrum(rows,capture_metadata(header)['phase_ticks'])}
     (folder/(name+'.json')).write_text(json.dumps({k:v for k,v in self.last_capture.items() if k!='rows'},indent=2))
     self.log('Capture saved · '+str(len(rows))+' CRC-checked rows · '+name)
    elif action=='motion_fit':
     from motion_calibration import fit_motion
     files=data.get('files',[]);anchors=data.get('anchors',[])
     if not isinstance(files,list) or not 24<=len(files)<=80 or len(set(files))!=len(files):raise ValueError('Record 24–80 distinct movement captures.')
     if not isinstance(anchors,list) or len(anchors)>8:raise ValueError('Invalid reference list.')
     receiver=None
     def saved_complex(name):
      nonlocal receiver
      if not isinstance(name,str) or not re.fullmatch(r'capture-\d{8}-\d{6}-\d{6}\.csv',name):raise ValueError('Invalid capture filename.')
      try:c=json.loads((ROOT/'captures'/(name+'.json')).read_text())
      except (OSError,json.JSONDecodeError):raise ValueError('Missing saved capture metadata.')
      if c.get('firmware') not in ('0.2.4-diagnostics','0.2.5-diagnostics','0.2.6-diagnostics','0.2.7-diagnostics') or c.get('device_port')!=self.port:raise ValueError('Capture belongs to another device or firmware.')
      if c['axes']!=7 or not c['mask'] or c['mask']&(c['mask']-1):raise ValueError('Capture must contain all three transmitters and one receiver.')
      if receiver is not None and receiver!=c['mask']:raise ValueError('Captures contain different receivers.')
      receiver=c['mask'];sp=c['spectrum'][str(receiver.bit_length())]
      if sp['clipped']:raise ValueError('A saved capture is clipped.')
      values=[t.get('xyz_source_counts') for t in sp['amplitudes']]
      if any(v is None for v in values):raise ValueError('A capture lacks the timing reference.')
      return values
     values=[saved_complex(f) for f in files];anchor_values=[];distances=[]
     for a in anchors:
      position=a.get('position_m',[])
      if len(position)!=3 or not all(isinstance(v,(float,int)) and math.isfinite(v) for v in position):raise ValueError('Invalid saved reference position.')
      distance=math.sqrt(sum(v*v for v in position))
      if not .04<=distance<=.3:raise ValueError('Invalid saved reference distance.')
      anchor_values.append(saved_complex(a['file']));distances.append(distance)
     # No transmitter or sensing power during numerical fitting.
     self.exchange('sensors off');self.refresh()
     self.motion_calibration=None;self.diagnostics.pop('motion_calibration',None)
     cal=fit_motion(values,anchor_values if len(anchor_values)>=2 else None,distances)
     cal['receiver_mask']=receiver;cal['files']=files;cal['anchors']=anchors
     folder=ROOT/'calibrations';folder.mkdir(exist_ok=True)
     from glove_tracking import register_to_references
     registration=register_to_references(cal,saved_complex)
     cal['published']=registration is not None and cal.get('metres_per_unit') is not None
     filename=('motion-' if cal['published'] else 'draft-motion-')+datetime.datetime.now().strftime('%Y%m%d-%H%M%S-%f')+'.json'
     output=folder/filename
     output.write_text(json.dumps(cal,indent=2))
     if json.loads(output.read_text())!=cal:raise OSError('Calibration readback failed.')
     self.motion_calibration=cal
     self.diagnostics['motion_calibration']={k:v for k,v in cal.items() if k not in ('files','anchors','gains','phase','zero_rotation')}
     self.diagnostics['motion_calibration']['file']=filename
    elif action=='motion_solve':
     from motion_calibration import motion_pose
     c=self.last_capture;cal=self.motion_calibration
     if not cal or not c or c['mask']!=cal['receiver_mask'] or c['axes']!=7:raise ValueError('Record and fit movement before requesting a pose.')
     sp=c['spectrum'][str(c['mask'].bit_length())]
     if sp['clipped']:raise ValueError('Clipped data; pose unavailable.')
     values=[t.get('xyz_source_counts') for t in sp['amplitudes']]
     if any(v is None for v in values):raise ValueError('Missing timing; pose unavailable.')
     self.diagnostics['motion_pose']=motion_pose(cal,values)
    elif action=='glove_load':
     from glove_tracking import register_to_references
     self.glove_calibrations={};self.pose_stabilizers={};profiles={};profile_errors=[]
     for f in sorted((ROOT/'calibrations').glob('motion-*.json'),reverse=True):
      try:
       cal=json.loads(f.read_text());mask=cal['receiver_mask'];sensor=str(mask.bit_length())
       if mask not in (1,2,4,8,16) or sensor in self.glove_calibrations:continue
       def saved(name):
        if not re.fullmatch(r'capture-\d{8}-\d{6}-\d{6}\.csv',name):raise ValueError('Invalid filename')
        c=json.loads((ROOT/'captures'/(name+'.json')).read_text())
        if c.get('device_port')!=self.port or c.get('firmware') not in ('0.2.4-diagnostics','0.2.5-diagnostics','0.2.6-diagnostics','0.2.7-diagnostics') or c.get('mask')!=mask:raise ValueError('Device mismatch')
        sp=c['spectrum'][sensor]
        if sp['clipped']:raise ValueError('Clipped calibration')
        values=[t.get('xyz_source_counts') for t in sp['amplitudes']]
        if any(v is None for v in values):raise ValueError('No phase reference')
        return values
       saved(cal['files'][0]);registration=register_to_references(cal,saved)
       self.glove_calibrations[sensor]=(cal,registration)
       profiles[sensor]={'file':f.name,'registration':registration,'held_out_rms':cal['held_out_rms']}
      except (ValueError,KeyError,OSError,TypeError) as e:profile_errors.append({'file':f.name,'reason':str(e)})
     self.diagnostics['glove_profile_errors']=profile_errors;self.diagnostics['glove_profiles']=profiles;self.diagnostics['glove_poses']={};self.diagnostics.pop('glove_solved_capture',None)
    elif action=='glove_flip':
     sensor=str(int(data.get('sensor',0)))
     if sensor not in self.pose_stabilizers:raise ValueError('No tracked receiver on this port.')
     self.pose_stabilizers[sensor].flip()
    elif action=='glove_solve':
     from glove_tracking import automatic_pose, signal_summary
     from pose_stabilizer import PoseStabilizer
     c=self.last_capture
     if not c or c['axes']!=7 or c['device_port']!=self.port:raise ValueError('Take a current all-axis capture first.')
     if self.diagnostics.get('glove_solved_capture')==c['file']:return self.state()
     poses={}
     for sensor,sp in c['spectrum'].items():
      try:
       if sp['clipped']:raise ValueError('Signal clipped; move farther away.')
       values=[t.get('xyz_source_counts') for t in sp['amplitudes']]
       if any(v is None for v in values):raise ValueError('Timing reference unavailable.')
       borrowed=sensor not in self.glove_calibrations
       if borrowed and not self.glove_calibrations:raise ValueError('No reference model; signal view only.')
       model_sensor=sensor if not borrowed else next(iter(self.glove_calibrations))
       cal,registration=self.glove_calibrations[model_sensor]
       tracker=self.pose_stabilizers.setdefault(sensor,PoseStabilizer());now=time.monotonic()
       solved=automatic_pose(cal,registration,values,previous_position=tracker.previous(now))
       poses[sensor]=tracker.apply(solved,now);poses[sensor]['capture']=c['file']
       poses[sensor]['provisional']=borrowed;poses[sensor]['model_port']=model_sensor
      except ValueError as e:poses[sensor]={'error':str(e),'provisional':sensor not in self.glove_calibrations}
      poses[sensor]['signal']=signal_summary(sp)

     self.diagnostics['glove_poses']=poses
     self.diagnostics['glove_solved_capture']=c['file']
    elif action=='pose_fit':
     from pose_calibration import calibrate
     references=data.get('references',[])
     if not isinstance(references,list) or not 8<=len(references)<=30:raise ValueError('Collect 8–30 reference positions.')
     samples=[];receiver=None;files=set()
     for ref in references:
      name=ref.get('file','')
      if not re.fullmatch(r'capture-\d{8}-\d{6}-\d{6}\.csv',name) or name in files:raise ValueError('Each reference needs a distinct saved capture.')
      files.add(name)
      try:c=json.loads((ROOT/'captures'/(name+'.json')).read_text())
      except (OSError,json.JSONDecodeError):raise ValueError('Reference capture metadata is missing.')
      if c.get('firmware') not in ('0.2.4-diagnostics','0.2.5-diagnostics','0.2.6-diagnostics','0.2.7-diagnostics') or c.get('device_port')!=self.port:raise ValueError('Reference belongs to another device or firmware.')
      if c['axes']!=7 or c['mask']&(c['mask']-1):raise ValueError('Reference must measure all transmitters and one fingertip.')
      if receiver is not None and receiver!=c['mask']:raise ValueError('References use different fingertips.')
      receiver=c['mask'];spec=c['spectrum'][str(receiver.bit_length())]
      if spec['clipped']:raise ValueError('A reference is clipped.')
      values=[t.get('xyz_source_counts') for t in spec['amplitudes']]
      if any(v is None for v in values):raise ValueError('Reference has no transmitter timing information.')
      samples.append({'position_m':ref['position_m'],'complex_counts':values})
     calibration=calibrate(samples);calibration['receiver_mask']=receiver
     calibration['references']=references;calibration['firmware']='0.2.4-diagnostics'
     folder=ROOT/'calibrations';folder.mkdir(exist_ok=True)
     filename='calibration-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S')+'.json'
     (folder/filename).write_text(json.dumps(calibration,indent=2));self.pose_calibration=calibration
     self.diagnostics['pose_calibration']={'file':filename,'held_out_rms':calibration['held_out_rms'],'permutation':calibration['permutation']}
    elif action=='pose_solve':
     from pose_calibration import track
     c=self.last_capture;cal=self.pose_calibration
     if not cal or not c or c['mask']!=cal['receiver_mask'] or c['axes']!=7:raise ValueError('Calibrate this receiver and take an all-axis capture first.')
     spec=c['spectrum'][str(c['mask'].bit_length())]
     if spec['clipped']:raise ValueError('Clipped measurement; pose rejected.')
     values=[t.get('xyz_source_counts') for t in spec['amplitudes']]
     if any(v is None for v in values):raise ValueError('Timing reference missing; pose rejected.')
     self.diagnostics['pose']=track(cal,values);self.diagnostics['pose']['capture']=c['file']
    elif action=='reset':
     self.exchange('sensors off');self.link.write(b'reset\n');self.link.flush();self.close();self.log('Hub restarting. Wait a moment, then connect again.')
    else:raise ValueError('Unsupported action.')
    return self.state()
   except (serial.SerialException,OSError) as e:
    self.close();self.log(str(e),'error');raise ValueError(str(e)) from e
def spectrum(rows,phase_ticks=None):
 # Magnitudes only: capture start is not a calibrated transmitter phase reference.
 result={}
 for sensor in sorted({r[0] for r in rows}):
  raw=[r for r in rows if r[0]==sensor]
  # Exclude the SYNC/filter startup interval. 192 retained samples contain
  # integer cycles of all three tones at 24 kSPS. Raw CSV remains untouched.
  skipped=64 if len(raw)==256 else 0
  block=raw[skipped:];n=len(block)
  result[str(sensor)]={'settling_samples_ignored':skipped,'analyzed_samples':n,'raw_clipped':any(abs(r[c])>=32760 for r in raw for c in (3,4,5)),'clipped':any(abs(r[c])>=32760 for r in block for c in (3,4,5)),'amplitudes':[]}
  for frequency in (4500,5625,6750):
   amps=[];phasors=[];referenced=[]
   tick=phase_ticks[0 if sensor<=3 else 1] if phase_ticks else 4294967295
   phase=2*math.pi*frequency*(tick/122880000+skipped/24000) if tick!=4294967295 else None
   for ch in (3,4,5):
    mean=sum(r[ch] for r in block)/n
    real=sum((r[ch]-mean)*math.cos(2*math.pi*frequency*k/24000) for k,r in enumerate(block))*2/n
    imag=sum((r[ch]-mean)*math.sin(2*math.pi*frequency*k/24000) for k,r in enumerate(block))*2/n
    amps.append(round(math.hypot(real,imag),3))
    phasors.append([round(real,6),round(-imag,6)])
    if phase is not None:
     c=complex(real,-imag)*complex(math.cos(phase),-math.sin(phase))
     referenced.append([round(c.real,6),round(c.imag,6)])
   result[str(sensor)]['amplitudes'].append({'hz':frequency,'xyz_counts':amps,'xyz_complex_counts':phasors,'phase_reference':'capture-local, not transmitter-referenced','xyz_source_counts':referenced or None,'source_reference_status':'DRDY interrupt timestamp; electrical phase and latency not calibrated' if referenced else 'unavailable'})
 return result
hub=Hub();atexit.register(hub.close)
def watchdog():
 while True:
  time.sleep(2)
  with hub.lock:
   if hub.link and time.monotonic()-hub.last_seen>45:
    hub.close();hub.log('No panel heartbeat for 45 seconds. USB disconnected and sensor power off requested.','warning')
class Handler(BaseHTTPRequestHandler):
 def log_message(self,*args):pass
 def send(self,status,data,kind='application/json'):
  if kind=='application/json':data=json.dumps(data).encode()
  elif isinstance(data,str):data=data.encode()
  self.send_response(status);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(data)
 def valid_host(self):return self.headers.get('Host') in (f'127.0.0.1:{PORT}',f'localhost:{PORT}')
 def do_GET(self):
  if not self.valid_host():return self.send(403,{'error':'Local access only'})
  path=urlparse(self.path).path
  if path in ('/unified','/unified.html'):return self.send(200,(ROOT/'unified.html').read_text().replace('__TOKEN__',TOKEN),'text/html; charset=utf-8')
  if path in ('/hand','/hand.html'):return self.send(200,(ROOT/'hand.html').read_text().replace('__TOKEN__',TOKEN),'text/html; charset=utf-8')
  if path in ('/motion','/motion.html'):return self.send(200,(ROOT/'motion.html').read_text().replace('__TOKEN__',TOKEN),'text/html; charset=utf-8')
  if path in ('/pose','/pose.html'):return self.send(200,(ROOT/'pose.html').read_text().replace('__TOKEN__',TOKEN),'text/html; charset=utf-8')
  if path in ('/calibration','/calibration.html'):return self.send(200,(ROOT/'calibration.html').read_text().replace('__TOKEN__',TOKEN),'text/html; charset=utf-8')
  if path in ('/axis','/axis.html'):return self.send(200,(ROOT/'axis.html').read_text().replace('__TOKEN__',TOKEN),'text/html; charset=utf-8')
  if path in ('/axis.js','/axis-model.js','/imu-view.js','/hand.js','/hand-orientation.js','/hand-translation.js','/hand-schedule.js','/robot-hand.js','/aero-kinematics.js','/cad-hand.mjs','/axis-guide.js','/hand-z-reflection.js'):return self.send(200,(ROOT/path[1:]).read_text(),'text/javascript; charset=utf-8')
  if path.startswith('/vendor/'):
   asset=(ROOT/path.lstrip('/')).resolve()
   if not asset.is_relative_to((ROOT/'vendor').resolve()) or not asset.is_file():return self.send(404,{'error':'Asset not found'})
   if asset.suffix=='.json':return self.send(200,json.loads(asset.read_text()))
   return self.send(200,asset.read_bytes(),'text/javascript; charset=utf-8' if asset.suffix=='.js' else 'application/octet-stream')
  if path=='/':return self.send(200,(ROOT/'index.html').read_text().replace('__TOKEN__',TOKEN),'text/html; charset=utf-8')
  if path=='/api/state':
   if self.headers.get('X-Control-Token')!=TOKEN:return self.send(403,{'error':'Reload the panel'})
   with hub.lock:
    hub.last_seen=time.monotonic()
    if hub.link and time.monotonic()-hub.last_poll>4:
     try:hub.refresh()
     except (serial.SerialException,OSError) as e:hub.close();hub.log(str(e),'error')
     hub.last_poll=time.monotonic()
    state=hub.state()
   return self.send(200,state)
  if path=='/api/ports':return self.send(200,hub.ports())
  if path.startswith('/captures/'):
   name=path.removeprefix('/captures/')
   if re.fullmatch(r'capture-\d{8}-\d{6}-\d{6}\.csv',name):
    file=ROOT/'captures'/name
    if file.is_file():return self.send(200,file.read_bytes(),'text/csv; charset=utf-8')
  return self.send(404,{'error':'Not found'})
 def do_POST(self):
  if not self.valid_host() or self.headers.get('X-Control-Token')!=TOKEN or self.headers.get('Origin') not in (f'http://127.0.0.1:{PORT}',f'http://localhost:{PORT}'):
   return self.send(403,{'error':'Open the local control panel to issue commands.'})
  if self.path=='/api/stop':
   try:return self.send(200,hub.emergency())
   except (OSError,serial.SerialException) as e:return self.send(400,{'error':str(e)})
  if self.path!='/api/action':return self.send(404,{'error':'Not found'})
  try:
   length=int(self.headers.get('Content-Length','0'))
   if not 0<length<32768:raise ValueError('Invalid request size')
   data=json.loads(self.rfile.read(length));state=hub.action(data.get('action'),data)
   self.send(200,state)
  except (ValueError,TypeError,KeyError) as e:self.send(400,{'error':str(e),'state':hub.state()})
if __name__=='__main__':
 threading.Thread(target=watchdog,daemon=True).start()
 print(f'6thSense control panel: http://127.0.0.1:{PORT}',flush=True)
 server=ThreadingHTTPServer(('127.0.0.1',PORT),Handler)
 try:server.serve_forever()
 except KeyboardInterrupt:pass
 finally:hub.close();server.server_close()

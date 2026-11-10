from pathlib import Path
import subprocess, os
root=Path(__file__).resolve().parents[1]
zig=Path(os.environ.get('ZIG_EXE', str(Path.home()/'.cache/bms-test-tools/ziglang/zig.exe')))
out=root/'build/tests';out.mkdir(parents=True,exist_ok=True)
src=root/'Hardware/BMS'
results=[]
for name,modules in [('test_bms',('bms_core','bms_config','bms_util','bq76930','ina226')),('test_comms',('bms_comms','bms_util')),('test_app',('bms_app','bms_core','bms_config','bms_util')),('test_storage',('bms_storage','bms_config','bms_util')),('test_display',('bms_display',)),('test_port_interlock',('bms_app','bms_core','bms_config','bms_util'))]:
    sources=[root/('tests/'+('test_app' if name=='test_port_interlock' else name)+'.c')]+[src/(n+'.c') for n in modules]
    executable=out/(name+'.exe')
    defines=['-DBMS_HARDWARE_VERIFIED=1','-DBMS_PORT_ARMING_VERIFIED=1'] if name=='test_app' else (['-DBMS_HARDWARE_VERIFIED=1'] if name=='test_port_interlock' else [])
    if name=='test_storage':defines+=['-I'+str(root/'tests/flash_mock')]
    subprocess.run([str(zig),'cc','-std=c99','-Wall','-Wextra','-Werror','-O2','-g',*defines,'-I'+str(src),*[str(x) for x in sources],'-o',str(executable)],check=True,cwd=root)
    p=subprocess.run([str(executable)],check=True,stdout=subprocess.PIPE,text=True,cwd=root);print(p.stdout,end='');results.append(p.stdout)
(out/'results.txt').write_text(''.join(results),encoding='utf-8')



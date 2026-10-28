import csv, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams.update({'pdf.fonttype':42,'font.family':'serif','font.serif':['Times New Roman','Liberation Serif','DejaVu Serif'],'font.size':8.5})
# Raw dashboard log; Runs A, B, C are runs 3, 5, 2. Plot 1 s before to 5 s after the climb starts.
rows=list(csv.DictReader(open('../../../data/pitch-runs.csv')))
def run_data(run):
    R=[r for r in rows if r['run']==run]
    t0=float(next(r for r in R if r['stage']=='Climb')['t_s'])
    R=[r for r in R if -1<=float(r['t_s'])-t0<=5]
    t=np.array([float(r['t_s'])-t0 for r in R])
    return t,np.array([float(r['pitch_fused_deg']) for r in R]),np.array([float(r['pitch_accel_deg']) for r in R]),[r['stage'] for r in R]
ink='#0b0b0b'; mute='#52514e'; grid='#e5e4e0'; acc='#eb6834'; fus='#2a78d6'
fig,axs=plt.subplots(3,1,figsize=(6.5,5.6),dpi=300,sharex=True)
for ax,(run,lab) in zip(axs,[('3','Run A'),('5','Run B'),('2','Run C')]):
    t,f,a,st=run_data(run)
    c=[i for i,s in enumerate(st) if s=='Climb']; t0,t1=t[c[0]],t[c[-1]]
    ax.axvspan(t0,t1,color='#f1f0ec',lw=0)
    ax.axhline(33.4,color=mute,lw=0.8,ls=(0,(4,3)))
    ax.plot(t,a,color=acc,lw=1.0,marker='o',ms=2.5,label='Accelerometer only')
    ax.plot(t,f,color=fus,lw=1.8,label='Fused (complementary filter)')
    ax.set_ylim(-35,160); ax.set_yticks([0,50,100,150])
    ax.text(0.01,0.93,lab,transform=ax.transAxes,fontsize=8.5,color=ink,va='top')
    ax.grid(True,color=grid,lw=0.6,axis='y'); ax.set_axisbelow(True)
    for s in ('top','right'): ax.spines[s].set_visible(False)
    for s in ('left','bottom'): ax.spines[s].set_color('#b5b4b0')
    ax.tick_params(colors=mute)
from matplotlib.patches import Patch
h,l=axs[0].get_legend_handles_labels()
h.append(Patch(facecolor='#f1f0ec',edgecolor='none')); l.append('Climbing')
axs[0].legend(h,l,frameon=False,fontsize=7,loc='upper right',bbox_to_anchor=(1.0,1.18),ncol=3,
              handlelength=1.6,handletextpad=0.5,columnspacing=1.2)
axs[0].text(4.95,33.4+2,'ramp, 33.4°',color=mute,fontsize=7,ha='right',va='bottom')
axs[1].set_ylabel('Pitch (°)')
axs[2].set_xlabel('Time from start of climb (s)')
axs[2].set_xlim(-1,5)
fig.tight_layout(); fig.savefig('../fig11_pitch.pdf')

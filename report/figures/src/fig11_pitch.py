import json, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
plt.rcParams.update({'pdf.fonttype':42,'font.family':'serif','font.serif':['Times New Roman','Liberation Serif','DejaVu Serif'],'font.size':8.5})
D=json.load(open('fig11.json'))
ink='#0b0b0b'; mute='#52514e'; grid='#e5e4e0'; acc='#eb6834'; fus='#2a78d6'
fig,axs=plt.subplots(3,1,figsize=(6.5,5.6),dpi=300,sharex=True)
for ax,(run,lab) in zip(axs,[('3','Run A'),('5','Run B'),('2','Run C')]):
    d=np.array([x[:3] for x in D[run]]); st=[x[3] for x in D[run]]
    t,f,a=d[:,0],d[:,1],d[:,2]
    c=[i for i,s in enumerate(st) if s=='C']; t0,t1=t[c[0]],t[c[-1]]
    ax.axvspan(t0,t1,color='#f1f0ec',lw=0)
    ax.axhline(33.4,color=mute,lw=0.8,ls=(0,(4,3)))
    ac=np.clip(a,-10,100)
    ax.plot(t,ac,color=acc,lw=1.0,marker='o',ms=2.5,label='Accelerometer only')
    hi=a>100
    ax.plot(t[hi],np.full(hi.sum(),100),'^',ms=4,color=acc)
    ax.plot(t,f,color=fus,lw=1.8,label='Fused (complementary filter)')
    ax.set_ylim(-12,108); ax.set_yticks([0,25,50,75,100]); ax.set_yticklabels(['0','25','50','75','100'])
    ax.text(0.01,0.93,lab,transform=ax.transAxes,fontsize=8.5,color=ink,va='top')
    ax.grid(True,color=grid,lw=0.6,axis='y'); ax.set_axisbelow(True)
    for s in ('top','right'): ax.spines[s].set_visible(False)
    for s in ('left','bottom'): ax.spines[s].set_color('#b5b4b0')
    ax.tick_params(colors=mute)
from matplotlib.patches import Patch
h,l=axs[0].get_legend_handles_labels()
h.append(Patch(facecolor='#f1f0ec',edgecolor='none')); l.append('Climbing')
from matplotlib.lines import Line2D
h.append(Line2D([],[],ls='none',marker='^',ms=4,color=acc)); l.append('Above 100° (clipped)')
axs[0].legend(h,l,frameon=False,fontsize=7,loc='upper right',bbox_to_anchor=(1.0,1.18),ncol=4,
              handlelength=1.6,handletextpad=0.5,columnspacing=1.2)
axs[0].text(4.95,33.4+2,'ramp, 33.4°',color=mute,fontsize=7,ha='right',va='bottom')
axs[1].set_ylabel('Pitch (°)')
axs[2].set_xlabel('Time from start of climb (s)')
axs[2].set_xlim(-1,5)
fig.tight_layout(); fig.savefig('../fig11_pitch.pdf')

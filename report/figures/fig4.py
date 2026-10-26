import math, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
g=9.81; th=math.radians(33.4); m=1.0; L=1.524; r=0.025; V=7.4; R=7.4/1.5
Ke=6.0/(5970*2*math.pi/60)
variants=[(12,500),(16,377),(26,226),(35,171),(60,103),(77,78),(130,45),(170,35),(280,21.3),(620,9.6),(1360,4.4)]
cand={170,280,620}
def Fs(G,eta=0.7): return 2*eta*G*Ke*V/R/r
def bis(f,a,b):
    for _ in range(100):
        c=(a+b)/2
        if f(a)*f(c)<=0: b=c
        else: a=c
    return c
def T(G,eta=0.7,mu=1.0,kr=1.0):
    F=Fs(G,eta); v0=V/(Ke*G)*r; Ft=mu*m*g*math.cos(th); W=kr*m*g*math.sin(th)
    if F<=W: return None
    if F>Ft: a1=(Ft-W)/m; v1=v0*(1-Ft/F); t1=v1/a1; x1=v1**2/(2*a1)
    else: v1=t1=x1=0
    vt=v0*(1-W/F); tau=m*v0/F
    return t1+bis(lambda t: x1+vt*t-tau*(vt-v1)*(1-math.exp(-t/tau))-L,0,1000)
plt.rcParams.update({'font.family':'serif','font.serif':['Times New Roman','Liberation Serif','DejaVu Serif'],'font.size':9,'mathtext.fontset':'dejavuserif'})
Gs=np.geomspace(4,520,400); ink='#0b0b0b'; mute='#52514e'; grid='#e5e4e0'; off='#a9a8a4'
fig,(a1,a2)=plt.subplots(2,1,figsize=(6.5,6.4),dpi=300,gridspec_kw={'height_ratios':[1,1.25]})
W=m*g*math.sin(th); Ft=[mu*m*g*math.cos(th) for mu in (0.8,1.0,1.2)]
a1.axhspan(0.5,W,color='#f2d0c4',lw=0); a1.axhspan(W,1.25*W,color='#fae8e1',lw=0); a1.axhspan(Ft[0],Ft[2],color='#d6e6f7',lw=0)
a1.plot(Gs,Fs(Gs),color=mute,lw=1.2)
for rpm,G in variants:
    c=rpm in cand
    a1.plot(G,Fs(G),'o',ms=6 if c else 5,color=ink if c else 'white',mec=ink if c else off,mew=1.2,zorder=3)
    a1.annotate(f'{rpm}',(G,Fs(G)),xytext=(5,-11),textcoords='offset points',fontsize=7.5,color=ink if c else mute)
a1.text(500,2.4,'Holding floor: cannot climb below this',ha='right',fontsize=7.5,color=mute,va='center')
a1.text(500,8.7,'Traction ceiling (Table 2)',ha='right',fontsize=7.5,color=mute,va='center')
a1.axhline(3*Ft[2],color=mute,lw=1,ls=(0,(4,3)))
a1.text(500,3*Ft[2]*0.8,'Traction ceiling × 3 (estimated FoS)',ha='right',fontsize=7.5,color=mute,va='center')
a1.set_yscale('log'); a1.set_ylim(0.8,700); a1.set_ylabel('Stall force (N)')
a1.set_yticks([1,2,5,10,20,50,100,200,500]); a1.set_yticklabels(['1','2','5','10','20','50','100','200','500'])
a1.set_title('(a) Stall force of each version against the bounds (filled: candidates)',fontsize=9,loc='left',color=ink)
cases=[(0.7,1.0,'#2a78d6','η = 0.7, $k_r$ = 1.0'),(0.7,1.25,'#eb6834','η = 0.7, $k_r$ = 1.25'),(0.6,1.25,'#1baf7a','η = 0.6, $k_r$ = 1.25')]
for eta,kr,c,lab in cases:
    a2.plot(Gs,[T(G,eta=eta,kr=kr) or np.nan for G in Gs],color=c,lw=1.6,label=lab)
    for rpm,G in variants:
        t=T(G,eta=eta,kr=kr)
        if t and t<60: a2.plot(G,t,'o',ms=5,color=c,mec='white',mew=1.0,zorder=3)
a2.plot(21.3,3.3,'s',ms=6.5,color=ink,zorder=4,label='Measured, 280 rpm')
for rpm,G in variants:
    t=T(G); y=(t*1.2 if t else 1.6)
    if rpm==620: y=2.3; G=G*0.93
    if rpm==280: y=2.0
    a2.annotate(f'{rpm}' if rpm!=1360 else '1360: does not climb',(G,y),ha=('right' if rpm==620 else 'center') if rpm!=1360 else 'left',fontsize=7.5,color=mute)
a2.set_yscale('log'); a2.set_ylim(1.4,60); a2.set_yticks([2,3,5,10,20,40]); a2.set_yticklabels(['2','3','5','10','20','40'])
a2.set_ylabel('Estimated climb time (s)')
a2.set_title('(b) Estimated time to climb the 1.524 m ramp',fontsize=9,loc='left',color=ink)
a2.legend(frameon=False,fontsize=8,loc='upper left',bbox_to_anchor=(0.40,1.0))
tk=[5,10,20,50,100,200,500]
for ax in (a1,a2):
    ax.set_xscale('log'); ax.set_xlim(4,520); ax.set_xticks(tk); ax.set_xticklabels([f'1:{t}' for t in tk]); ax.minorticks_off()
    ax.set_xlabel('Gear ratio (labels: rated rpm of each version on sale)')
    ax.grid(True,color=grid,lw=0.6); ax.set_axisbelow(True)
    for s in ('top','right'): ax.spines[s].set_visible(False)
    for s in ('left','bottom'): ax.spines[s].set_color('#b5b4b0')
    ax.tick_params(colors=mute)
a1.minorticks_off(); a2.minorticks_off()
a1.set_yticks([1,2,5,10,20,50,100,200,500]); a1.set_yticklabels(['1','2','5','10','20','50','100','200','500'])
a2.set_yticks([2,3,5,10,20,40]); a2.set_yticklabels(['2','3','5','10','20','40'])
fig.tight_layout(); fig.savefig('fig4_v4.png')
if __name__=='__main__':
    for rpm,G in variants: print(rpm, round(Fs(G),1), [round(T(G,eta=e,kr=k),2) if T(G,eta=e,kr=k) else None for e,k in ((0.7,1.0),(0.7,1.25),(0.6,1.25))])

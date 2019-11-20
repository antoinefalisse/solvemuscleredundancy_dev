global np
import numpy as np

def HillModel_RigidTendon(a,lMT,vMT,Parameters,ActiveFVParameters,PassiveFLParameters,Faparam):
    Fmax = Parameters[0]
    lMopt = Parameters[1]
    lTs = Parameters[2]
    alphaopt = Parameters[3]
    vMmax  = Parameters[4]
    
    # Hill-type muscle model: geometric relationships    
    w = np.multiply(lMopt,np.sin(alphaopt))
    lM = np.sqrt(np.square(lMT-lTs)+np.square(w)) # Rigid Tendon: lT = lTs
    lMtilde = np.divide(lM,lMopt)    
    lT = np.multiply(lTs,np.ones((len(lMtilde),1)))
    cos_alpha = np.divide((lMT-lT),lM);
    
    # Active muscle force-length relationship
    b11 = Faparam[0]
    b21 = Faparam[1]
    b31 = Faparam[2]
    b41 = Faparam[3]
    b12 = Faparam[4]
    b22 = Faparam[5]
    b32 = Faparam[6]
    b42 = Faparam[7]
    
    b13 = 0.1
    b23 = 1
    b33 = 0.5*np.sqrt(0.5)
    b43 = 0
    num3 = lMtilde-b23
    den3 = b33+b43*lMtilde    
    FMtilde3 = b13*np.exp(-0.5*(np.divide(np.square(num3),np.square(den3))))
    
    num1 = lMtilde-b21
    den1 = b31+b41*lMtilde
    FMtilde1 = b11*np.exp(-0.5*(np.divide(np.square(num1),np.square(den1))))
    
    num2 = lMtilde-b22;
    den2 = b32+b42*lMtilde;
    FMtilde2 = b12*np.exp(-0.5*(np.divide(np.square(num2),np.square(den2))))
    
    FMactFL = FMtilde1+FMtilde2+FMtilde3;   
    
    # Active muscle force-velocity relationship
    vMtilde = np.multiply(np.divide(vMT,vMmax),cos_alpha)
    e1 = 1.475*ActiveFVParameters[0]
    e2 = 0.25*ActiveFVParameters[1]
    e3 = ActiveFVParameters[2] + 0.75
    e4 = ActiveFVParameters[3] - 0.027
    
    FMactFV = e1*np.log((e2*vMtilde+e3)+np.sqrt(np.square(e2*vMtilde+e3)+1))+e4
    
    # Active muscle force
    FMact = np.multiply(np.multiply(a,FMactFL),FMactFV)
    
    # Passive muscle force-length relationship
    e0 = 0.6
    kpe = 4
    t5 = np.exp(kpe * (lMtilde - 1) / e0)
    FMpas = np.divide(((t5-1)-PassiveFLParameters[0]),PassiveFLParameters[1])
    
    # Muscle force
    FM_norm = (FMact+FMpas)
    FM = np.multiply(Fmax,FM_norm)
    
    return FM, lMtilde, FMactFL, FMactFV, FMpas, cos_alpha
    
def main():
    
    Faparam = np.array([[0.814483478343008],[1.055033428970575],[0.162384573599574],[0.063303448465465],[0.433004984392647],[0.716775413397760],[-0.029947116970696],[0.200356847296188]]);
    ActiveFVParameters = np.array([-0.215812499592629,-32.596624173901000,-1.124121508647863,0.912644059915004])
    PassiveFLParameters = np.array([-0.995172050006169,53.598150033144236])
    
    Parameters = np.array([[2.5],[4.5],[6.5],[8.5],[10.5]])
    lMT = np.array([[1.5],[3.5],[5.5],[7.5],[9.5]])
    vMT = np.array([[1.5],[3.5],[5.5],[7.5],[9.5]])
    a = np.array([[1.5],[3.5],[5.5],[7.5],[9.5]])
    
    [FM, lMtilde, FMactFL, FMactFV, FMpas, cos_alpha] = HillModel_RigidTendon(a,lMT,vMT,Parameters,ActiveFVParameters,PassiveFLParameters,Faparam)
    
    print(FM)
    
main()
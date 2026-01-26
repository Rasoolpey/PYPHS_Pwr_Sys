"""
IM Analysis Library (Extracted Core Functions)
Based on IM_analysis_lib_v1.py by YLI @ Energinet
"""
import numpy as np
import numpy.linalg as lnlg

# --- Time Domain Tools ---

def DFT_1f(signal, fs, f):
    """
    DFT analysis at a single selected frequency.
    Exact implementation from IMTB.
    """
    N = len(signal)
    T_signal = N/fs
    f_nyquist = fs/2

    if f >= f_nyquist:
        return 0
    elif f == 0:
        return sum(signal)/N
    elif f < 0:
        # Negative frequency handling
        fpos = -f
        delta_t = 1/fs
        t = np.linspace(delta_t, T_signal, N)
        s_m = np.exp(-1j*2*np.pi*fpos*t)
        X = np.sum(signal * s_m)/N
        return np.conj(X)
    
    # Positive frequency
    delta_t = 1/fs
    t = np.linspace(delta_t, T_signal, N)
    s_m = np.exp(-1j*2*np.pi*f*t)
    X = np.sum(signal * s_m)/N
    return X

# --- Frequency Domain Tools ---

def calc_MIMO_Z(V_d_1, V_q_1, I_d_1, I_q_1, 
                V_d_2, V_q_2, I_d_2, I_q_2):
    """
    Calculates 2x2 Impedance Matrix from two injection scenarios.
    
    Scenario 1: D-axis injection
    Scenario 2: Q-axis injection
    
    Returns: [[Zdd, Zdq], [Zqd, Zqq]]
    """
    # Construct Voltage Matrix [Col1=Inj1, Col2=Inj2]
    V_mat = np.array([
        [V_d_1, V_d_2],
        [V_q_1, V_q_2]
    ])
    
    # Construct Current Matrix
    I_mat = np.array([
        [I_d_1, I_d_2],
        [I_q_1, I_q_2]
    ])
    
    # Z = V * I^-1
    try:
        Z_mat = np.matmul(V_mat, lnlg.inv(I_mat))
    except lnlg.LinAlgError:
        Z_mat = np.array([[np.nan, np.nan], [np.nan, np.nan]])
        
    return Z_mat
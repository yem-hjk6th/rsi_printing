import os

base_dir = os.path.dirname(os.path.abspath(__file__))

x_start = 850
x_end = 900
x_trans = 855
y_starts = [-183, -283, -383, -483, -583]
y_offset = -50

z_first = -10.0
layer_height = 1.5
num_layers = 10

for file_idx in range(5):
    y_s = y_starts[file_idx]
    y_e = y_s + y_offset
    L = []
    L.append('DEF RSI_4pt ( )')
    L.append('')
    L.append(';VARIABLE DECLARATIONS')
    L.append('DECL INT ret')
    L.append('DECL INT CONTID')
    L.append('')
    L.append(';FOLD INI')
    L.append('BAS (#INITMOV,0 )')
    L.append(';BASE IS 0, TOOL IS 3, FIXED START POS')
    L.append('BAS(#TOOL, 3)')
    L.append('BAS(#BASE, 0)')
    L.append('BAS(#VEL_PTP, 10)')
    L.append('PDAT_ACT.ACC = 50')
    L.append('PDAT_ACT.APO_DIST = 50')
    L.append('$BWDSTART = TRUE')
    L.append('$VEL.CP=0.2')
    L.append('$ADVANCE=3')
    L.append(';ENDFOLD (INI)')
    L.append('')
    L.append('PTP  {A1 5,A2 -90,A3 100,A4 5,A5 -10,A6 -5,E1 0,E2 0,E3 0,E4 0}')
    L.append('')
    L.append(';EXTRUDER RPM')
    L.append('PelletExtruderRPM = 400')
    L.append('')
    L.append(';RSI INI')
    L.append('ret = RSI_CREATE("RSI_MIN.rsi", CONTID, TRUE)')
    L.append('IF (ret <> RSIOK) THEN')
    L.append('    HALT')
    L.append('ENDIF')
    L.append('')
    L.append('ret = RSI_ON(#RELATIVE)')
    L.append('IF (ret <> RSIOK) THEN')
    L.append('    HALT')
    L.append('ENDIF')
    L.append('')
    L.append(';MAIN')
    L.append(f"PTP {{X {x_start}, Y {y_s}, Z 28, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0, S 'B110'}} C_PTP")
    L.append(f'LIN {{X {x_start}, Y {y_s}, Z -7.5, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0}} ')
    L.append('$VEL.CP=0.01')

    for layer in range(num_layers):
        z = z_first + layer * layer_height
        zs = f'{z:.1f}'
        L.append(f';LAYER {layer+1}')
        L.append(f'LIN {{X {x_start}, Y {y_s}, Z {zs}, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0}} C_DIS')
        L.append(f'LIN {{X {x_start}, Y {y_e}, Z {zs}, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0}} C_DIS')
        L.append(f'LIN {{X {x_end}, Y {y_e}, Z {zs}, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0}} C_DIS')
        L.append(f'LIN {{X {x_end}, Y {y_s}, Z {zs}, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0}} C_DIS')
        if layer < num_layers - 1:
            # Transition point: at X=855 (5mm from start), still at current Z
            L.append(f'LIN {{X {x_trans}, Y {y_s}, Z {zs}, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0}} C_DIS')
        else:
            # Last layer: return to point 1
            L.append(f'LIN {{X {x_start}, Y {y_s}, Z {zs}, A 0, B 90, C -0, E1 0, E2 0, E3 0, E4 0}} C_DIS')

    L.append('')
    L.append('ret = RSI_OFF()')
    L.append('')
    L.append(';RETURN TO HOME')
    L.append('PTP  {A1 5,A2 -90,A3 100,A4 5,A5 -10,A6 -5,E1 0,E2 0,E3 0,E4 0}')
    L.append('')
    L.append('END')

    fn = f'exp08_cube_{file_idx+1}.src'
    fp = os.path.join(base_dir, fn)
    with open(fp, 'w', newline='\r\n') as f:
        f.write('\n'.join(L))
    print(f'Created {fn}')

print('Done - all 5 files generated.')

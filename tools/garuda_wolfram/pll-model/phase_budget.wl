(* Assumes motor_params.wl already loaded (provides omegaElec). *)

phiLagDeg[erpm_, tauSec_] := ArcTan[omegaElec[erpm] tauSec]/Degree;

(* comp forms (tau passed explicitly — no hidden global):
   "none"     -> 0
   "exact"    -> gain * arctan(w tau)   (gain = amp fraction, 1.0 = full cancel)
   "modelled" -> gain * erpm^2, capped at the raw lag (firmware K/period^2 law:
                 period ~ 1/erpm so 1/period^2 ~ erpm^2). Cannot over-cancel. *)
phiCompDeg[erpm_, form_, gain_, tauSec_] := Switch[form,
  "none", 0.,
  "exact", gain (ArcTan[omegaElec[erpm] tauSec]/Degree),
  "modelled", Min[gain erpm^2, ArcTan[omegaElec[erpm] tauSec]/Degree],
  _, 0.
];

advDeg[erpm_, advMax_, startErpm_, fullErpm_] := Which[
  erpm >= fullErpm, N[advMax],
  erpm > startErpm, N[advMax (erpm - startErpm)/(fullErpm - startErpm)],
  True, 0.
];

residualLagDeg[erpm_, tauSec_, form_, gain_] :=
  phiLagDeg[erpm, tauSec] - phiCompDeg[erpm, form, gain, tauSec];

(* smallest eRPM where residual lag exceeds advance headroom by marginDeg.
   Scan a grid for sign changes then refine with FindRoot; return Missing if never crossed. *)
wallErpm[tauSec_, form_, gain_, marginDeg_, advMax_, startErpm_, fullErpm_] :=
  Module[{f, grid, i, hit, eRPMlo, eRPMhi},
    f[e_] := residualLagDeg[e, tauSec, form, gain]
             - advDeg[e, advMax, startErpm, fullErpm] - marginDeg;
    grid = Range[20000, 300000, 1000];
    (* Find first zero crossing (negative to positive) *)
    hit = Missing["NoWall"];
    Do[
      If[i < Length[grid] && Sign[f[grid[[i]]]] != Sign[f[grid[[i+1]]]] &&
         f[grid[[i]]] < 0 && f[grid[[i+1]]] > 0,
        hit = {grid[[i]], grid[[i+1]]};
        Break[]
      ],
      {i, 1, Length[grid] - 1}
    ];
    If[MissingQ[hit], Return[hit]];
    eRPMlo = hit[[1]]; eRPMhi = hit[[2]];
    erpmSol /. FindRoot[f[erpmSol] == 0, {erpmSol, eRPMlo, eRPMhi}]
  ];

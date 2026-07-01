(* Assumes motor_params.wl + phase_budget.wl loaded. *)

Options[fitTauMargin] = {"FixMarginDeg" -> Automatic};

(* anchors: list of {label, observedWallErpm}. Fit tau (and margin unless fixed)
   so predicted wall matches observed. Reports whether ONE tau explains all
   anchors (max residual < 8000 eRPM = "oneTauExplains"). *)
fitTauMargin[anchors_List, form_, gain_, advMax_, startErpm_, fullErpm_,
             OptionsPattern[]] :=
  Module[{obs, predict, cost, sol, tauFit, marginFit, fixM, resid},
    obs = anchors[[All, 2]];
    fixM = OptionValue["FixMarginDeg"];
    predict[tau_?NumericQ, m_?NumericQ] :=
      wallErpm[tau, form, gain, m, advMax, startErpm, fullErpm];
    cost[tau_?NumericQ, m_?NumericQ] :=
      Total[(Replace[predict[tau, m], _Missing -> 1.*^6] - #)^2 & /@ obs];
    sol = If[fixM === Automatic,
      NMinimize[{cost[t, mm], 5.*^-6 <= t <= 80.*^-6, 0 <= mm <= 20}, {t, mm}],
      NMinimize[{cost[t, fixM], 5.*^-6 <= t <= 80.*^-6}, {t}]
    ];
    tauFit = t /. sol[[2]];
    marginFit = If[fixM === Automatic, mm /. sol[[2]], fixM];
    resid = (predict[tauFit, marginFit] - #) & /@ obs;
    <|"tauSec" -> tauFit, "marginDeg" -> marginFit,
      "residualsErpm" -> resid,
      "oneTauExplains" -> (Max[Abs[Replace[resid, _Missing -> 1.*^6, 1]]] < 8000)|>
  ];

(* Rank levers by the wall eRPM each buys, from the fitted baseline. *)
leverTable[fit_Association, motor_String] :=
  Module[{p = MotorParams[motor], tau = fit["tauSec"], m = fit["marginDeg"], base},
    base = wallErpm[tau, "modelled", 0, m, p["advMax"], p["advStartErpm"], p["advFullErpm"]];
    Dataset[<|
      "baseline" -> base,
      "halveTau" -> wallErpm[tau/2, "modelled", 0, m, p["advMax"], p["advStartErpm"], p["advFullErpm"]],
      "exactComp" -> wallErpm[tau, "exact", 1.0, m, p["advMax"], p["advStartErpm"], p["advFullErpm"]],
      "advPlus10" -> wallErpm[tau, "modelled", 0, m, p["advMax"] + 10, p["advStartErpm"], p["advFullErpm"]],
      "virtualNeutral_marginPlus5" -> wallErpm[tau, "modelled", 0, m + 5, p["advMax"], p["advStartErpm"], p["advFullErpm"]]
    |>]
  ];

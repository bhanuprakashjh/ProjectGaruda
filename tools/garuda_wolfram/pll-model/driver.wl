dir = DirectoryName[$InputFileName];
Get[FileNameJoin[{dir, "motor_params.wl"}]];
Get[FileNameJoin[{dir, "phase_budget.wl"}]];
Get[FileNameJoin[{dir, "calibrate.wl"}]];
Get[FileNameJoin[{dir, "observer_design.wl"}]];

runAnalysis[] := Module[{p, anchors, fit, levers, plot, plotPath},
  p = MotorParams["VEX"];
  (* Bench anchors from spec 2 / plan Global Constraints. The 80k point is
     pre-comp (form "none"); 150k/214k are with modelled comp. We fit tau on the
     modelled-comp regime using the two comp'd points, margin free. *)
  anchors = {{"150k", 150000}, {"214k", 214000}};
  fit = fitTauMargin[anchors, "modelled", 0, p["advMax"],
          p["advStartErpm"], p["advFullErpm"]];
  levers = leverTable[fit, "VEX"];
  plot = Plot[{phiLagDeg[e, fit["tauSec"]],
               advDeg[e, p["advMax"], p["advStartErpm"], p["advFullErpm"]]},
    {e, 20000, 260000},
    PlotLegends -> {"sense lag arctan(wt)", "available advance"},
    AxesLabel -> {"eRPM", "elec deg"}];
  plotPath = FileNameJoin[{dir, "wall_plot.png"}];
  Export[plotPath, plot];
  Print["oneTauExplains (front-end-limited?): ", fit["oneTauExplains"]];
  Print["fitted tau (us): ", fit["tauSec"] 1.*^6];
  <|"fit" -> fit, "levers" -> levers, "wallPlotPath" -> plotPath|>
];

If[$ScriptCommandLine =!= {} && Last[$ScriptCommandLine] === "run",
  Print[runAnalysis[]["levers"]]];

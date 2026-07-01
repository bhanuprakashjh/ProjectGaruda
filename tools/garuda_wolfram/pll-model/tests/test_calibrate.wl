dir = DirectoryName[$InputFileName];
Get[FileNameJoin[{dir, "..", "motor_params.wl"}]];
Get[FileNameJoin[{dir, "..", "phase_budget.wl"}]];
Get[FileNameJoin[{dir, "..", "calibrate.wl"}]];

(* Synthetic recovery: pick a known tau + margin, generate the wall it predicts
   with "none" comp, then confirm the fitter recovers tau within tolerance. *)
tauTrue = 28.*^-6; marginTrue = 3.;
wTrue = wallErpm[tauTrue, "none", 0, marginTrue, 22, 45000, 85000];
fit = fitTauMargin[{{"w", wTrue}}, "none", 0, 22, 45000, 85000,
        "FixMarginDeg" -> marginTrue];

report = TestReport[{
  VerificationTest[fit["tauSec"], tauTrue, SameTest -> (Abs[#1 - #2] < 2.*^-6 &),
    TestID -> "recover-tau"],
  (* residuals association is present and numeric *)
  VerificationTest[NumberQ[fit["marginDeg"]], True, TestID -> "margin-numeric"]
}];
Print[report["TestResultsDataset"]];
If[report["AllTestsSucceeded"], Print["ALL PASS"], Print["FAIL"]; Exit[1]];

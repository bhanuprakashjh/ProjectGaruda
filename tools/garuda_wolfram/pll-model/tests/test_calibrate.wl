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

(* leverTable on the VEX bench fit (same anchors/regime as driver.wl): confirm
   the baseline key is present and that exactComp's wallErpm is Missing (full
   comp cancels the lag entirely -> no wall), exercising the Missing-delta path. *)
pVex = MotorParams["VEX"];
fitVex = fitTauMargin[{{"150k", 150000}, {"214k", 214000}}, "modelled", 0,
           pVex["advMax"], pVex["advStartErpm"], pVex["advFullErpm"]];
levers = leverTable[fitVex, "VEX"];

report = TestReport[{
  VerificationTest[fit["tauSec"], tauTrue, SameTest -> (Abs[#1 - #2] < 2.*^-6 &),
    TestID -> "recover-tau"],
  (* residuals association is present and numeric *)
  VerificationTest[NumberQ[fit["marginDeg"]], True, TestID -> "margin-numeric"],
  VerificationTest[KeyExistsQ[Normal[levers], "baseline"], True,
    TestID -> "lever-has-baseline"],
  (* deltaErpm has the right shape: either a real number or Missing *)
  VerificationTest[
    Or[NumberQ[Normal[levers]["halveTau"]["deltaErpm"]],
       MissingQ[Normal[levers]["halveTau"]["deltaErpm"]]],
    True, TestID -> "lever-delta-shape"],
  VerificationTest[MissingQ[Normal[levers]["exactComp"]["wallErpm"]], True,
    TestID -> "lever-exactcomp-no-wall"]
}];
Print[report["TestResultsDataset"]];
If[report["AllTestsSucceeded"], Print["ALL PASS"], Print["FAIL"]; Exit[1]];

dir = DirectoryName[$InputFileName];
Get[FileNameJoin[{dir, "..", "motor_params.wl"}]];
Get[FileNameJoin[{dir, "..", "phase_budget.wl"}]];

approx[tol_] := (Abs[#1 - #2] < tol &);

report = TestReport[{
  (* arctan(2pi*2500*30e-6)=arctan(0.4712)=0.4405 rad = 25.24 deg *)
  VerificationTest[phiLagDeg[150000, 30.*^-6], 25.24, SameTest -> approx[0.1],
    TestID -> "lag-150k"],
  VerificationTest[phiLagDeg[80000, 30.*^-6], 14.10, SameTest -> approx[0.1],
    TestID -> "lag-80k"],
  (* lag is strictly increasing in eRPM *)
  VerificationTest[phiLagDeg[160000, 30.*^-6] > phiLagDeg[150000, 30.*^-6], True,
    TestID -> "lag-monotone"],
  (* advance ramp: 0 below start, max above full, midpoint linear *)
  VerificationTest[advDeg[40000, 22, 45000, 85000], 0., SameTest -> approx[1.*^-6],
    TestID -> "adv-below-start"],
  VerificationTest[advDeg[85000, 22, 45000, 85000], 22., SameTest -> approx[1.*^-6],
    TestID -> "adv-at-full"],
  VerificationTest[advDeg[65000, 22, 45000, 85000], 11., SameTest -> approx[0.01],
    TestID -> "adv-midpoint"],
  (* no comp, margin 0: wall is where arctan-lag == advMax(22). Should land in a
     physically sane band 100k..170k for tau=30us. *)
  VerificationTest[100000 < wallErpm[30.*^-6, "none", 0, 0, 22, 45000, 85000] < 170000,
    True, TestID -> "wall-none-band"],
  (* exact comp with full amp cancels lag -> no finite wall from lag term:
     residual ~ 0, so wallErpm returns Missing (no desync from lag). *)
  VerificationTest[Head[wallErpm[30.*^-6, "exact", 1.0, 0, 22, 45000, 85000]],
    Missing, TestID -> "wall-exact-none"]
}];
Print[report["TestResultsDataset"]];
If[report["AllTestsSucceeded"], Print["ALL PASS"], Print["FAIL"]; Exit[1]];

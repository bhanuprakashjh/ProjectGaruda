Get[FileNameJoin[{DirectoryName[$InputFileName], "..", "motor_params.wl"}]];

report = TestReport[{
  VerificationTest[MotorParams["VEX"]["PP"], 6, TestID -> "vex-polepairs"],
  VerificationTest[MotorParams["VEX"]["KeUvSRad"], 230, TestID -> "vex-ke"],
  (* 0.44 ohm phase-to-phase => 0.22 ohm per phase *)
  VerificationTest[MotorParams["VEX"]["RsOhm"], 0.22, TestID -> "vex-rs-per-phase"],
  VerificationTest[MotorParams["VEX"]["tauSec"], 30.*^-6, TestID -> "vex-tau"],
  VerificationTest[MotorParams["Hurst300"]["shape"], "sine", TestID -> "hurst-sine"],
  (* omegaElec: 150000 eRPM = 2500 elec Hz => 2 Pi 2500 rad/s *)
  VerificationTest[omegaElec[150000], 2. Pi 2500, SameTest -> (Abs[#1 - #2] < 1.*^-6 &),
    TestID -> "omega-150k"]
}];
Print[report["TestResultsDataset"]];
If[report["AllTestsSucceeded"], Print["ALL PASS"], Print["FAIL"]; Exit[1]];

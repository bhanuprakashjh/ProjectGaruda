dir = DirectoryName[$InputFileName];
Get[FileNameJoin[{dir, "..", "motor_params.wl"}]];
Get[FileNameJoin[{dir, "..", "observer_design.wl"}]];

des = bemfObserverGains["VEX", 150000, {-8000., -8500., -9000.}];
eig = Sort[Re[Eigenvalues[des["A"] - des["L"].des["C"]]]];

report = TestReport[{
  VerificationTest[Dimensions[des["L"]], {3, 1}, TestID -> "L-shape"],
  (* placed poles recovered (real parts) within tolerance *)
  VerificationTest[eig, Sort[{-8000., -8500., -9000.}],
    SameTest -> (Max[Abs[#1 - #2]] < 5. &), TestID -> "poles-placed"],
  (* observer error dynamics stable: all real parts negative *)
  VerificationTest[Max[Re[Eigenvalues[des["A"] - des["L"].des["C"]]]] < 0, True,
    TestID -> "stable"]
}];
Print[report["TestResultsDataset"]];
If[report["AllTestsSucceeded"], Print["ALL PASS"], Print["FAIL"]; Exit[1]];

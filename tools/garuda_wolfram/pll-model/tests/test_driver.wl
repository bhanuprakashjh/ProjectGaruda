dir = DirectoryName[$InputFileName];
Get[FileNameJoin[{dir, "..", "driver.wl"}]];

res = runAnalysis[];
report = TestReport[{
  VerificationTest[KeyExistsQ[res, "fit"], True, TestID -> "has-fit"],
  VerificationTest[KeyExistsQ[res, "levers"], True, TestID -> "has-levers"],
  VerificationTest[BooleanQ[res["fit"]["oneTauExplains"]], True,
    TestID -> "verdict-boolean"],
  VerificationTest[FileExistsQ[res["wallPlotPath"]], True, TestID -> "plot-written"]
}];
Print[report["TestResultsDataset"]];
If[report["AllTestsSucceeded"], Print["ALL PASS"], Print["FAIL"]; Exit[1]];

Quiet[Needs["ControlSystems`"], {Get::noopen, Needs::nocont}];
(* Assumes motor_params.wl loaded. *)

bemfObserverGains[motor_String, erpmOp_, poles_List] :=
  Module[{p = MotorParams[motor], r, l, we, aA, cC, ssDual, kGain, lGain},
    r = p["RsOhm"]; l = p["LsH"]; we = omegaElec[erpmOp];
    aA = {{-r/l, -1/l, 0}, {0, 0, 1}, {0, -we^2, 0}};
    cC = {{1, 0, 0}};
    (* Observer poles via state feedback on the dual system (A^T, C^T). *)
    ssDual = StateSpaceModel[{Transpose[aA], Transpose[cC]}];
    kGain = StateFeedbackGains[ssDual, poles];
    lGain = Transpose[kGain];
    <|"L" -> lGain, "A" -> aA, "C" -> cC, "poles" -> poles|>
  ];

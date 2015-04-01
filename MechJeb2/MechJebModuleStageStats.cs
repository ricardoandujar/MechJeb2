﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using System.Threading;
using KerbalEngineer.VesselSimulator;

namespace MuMech
{
    //Other modules can request that the stage stats be computed by calling RequestUpdate
    //This module will then run the stage stats computation in a separate thread, update
    //the publicly available atmoStats and vacStats. Then it will disable itself unless
    //it got another RequestUpdate in the meantime.
    public class MechJebModuleStageStats : ComputerModule
    {
        public MechJebModuleStageStats(MechJebCore core) : base(core) { }

        [ToggleInfoItem("ΔV include cosine losses", InfoItem.Category.Thrust, showInEditor = true)]
        public bool dVLinearThrust = true;

        public Stage atmLastStage { get; private set; }
        public Stage vacLastStage { get; private set; }

        public Stage[] atmoStats = {};
        public Stage[] vacStats = {};

        private bool resultWaiting = false;

        public CelestialBody editorBody;
        private CelestialBody simBody;

        public override void OnStart(PartModule.StartState state)
        {
            SimManager.UpdateModSettings();
            SimManager.OnReady -= this.GetStageInfo;
            SimManager.OnReady += this.GetStageInfo;
            base.OnAwake();
        }

        private void GetStageInfo()
        {
            resultWaiting = true;
        }

        public void RequestUpdate(object controller)
        {
            users.Add(controller);
            updateRequested = true;

            if (HighLogic.LoadedSceneIsEditor) TryStartSimulation();
        }

        private bool updateRequested = false;

        public override void OnFixedUpdate()
        {
            TryStartSimulation();
        }

        public void TryStartSimulation()
        {
            if (resultWaiting && SimManager.ResultsReady())  // <--- is the ResultsReady necessary ?
            {
                atmLastStage = SimManager.LastAtmStage;
                vacLastStage = SimManager.LastVacStage;

                atmoStats = SimManager.AtmStages;
                vacStats = SimManager.VacStages;

                resultWaiting = false;
            }

            if ((HighLogic.LoadedSceneIsEditor || vessel.isActiveVessel) && SimManager.ResultsReady())
            {
                if (updateRequested)
                {
                    updateRequested = false;
                    StartSimulation();
                }
                else
                {
                    users.Clear();
                }
            }
        }

        protected void StartSimulation()
        {
            simBody = HighLogic.LoadedSceneIsEditor ? editorBody ?? Planetarium.fetch.Home : vessel.mainBody;
            SimManager.Gravity = 9.81 * simBody.GeeASL;

            SimManager.Atmosphere = simBody.atmosphere ? 101.325 * simBody.atmosphereMultiplier : 0;
            SimManager.Velocity = HighLogic.LoadedSceneIsEditor ? 0 : vessel.srfSpeed;
            SimManager.vectoredThrust = dVLinearThrust;

            Profiler.BeginSample("MechJebModuleStageStats.StartSimulation()");

            SimManager.RequestSimulation();
            SimManager.TryStartSimulation();
            Profiler.EndSample();
        }
    }
}

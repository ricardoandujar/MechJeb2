﻿using System;
using JetBrains.Annotations;
using KSP.Localization;
using UnityEngine;
using static MechJebLib.Utils.Statics;
using Object = UnityEngine.Object;

namespace MuMech
{
    [UsedImplicitly]
    public class MechJebModuleFlightRecorderGraph : DisplayModule
    {
        private const int ScaleTicks = 11;

        public struct graphState
        {
            public double   minimum;
            public double   maximum;
            public string[] labels;
            public double[] labelsPos;
            public int      labelsActive;
            public bool     display;

            public void Reset()
            {
                minimum   = 0;
                maximum   = 0;
                labels    = new string[ScaleTicks];
                labelsPos = new double[ScaleTicks];
            }
        }

        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool downrange = true;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool realAtmo;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool stages;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public int hSize = 4;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public int vSize = 2;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public bool autoScale = true;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public int timeScale;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public int downrangeScale;

        [Persistent(pass = (int)Pass.GLOBAL)]
        public int scaleIdx;

        public bool ascentPath = false;

        private static          Texture2D     backgroundTexture;
        private                 CelestialBody oldMainBody;
        private static readonly int           typeCount = Enum.GetValues(typeof(MechJebModuleFlightRecorder.recordType)).Length;

        private readonly graphState[] graphStates;
        private          double       lastMaximumAltitude;
        private readonly double       precision = 0.2;

        private int width  = 512;
        private int height = 256;

        private bool paused;

        private float hPos;

        private bool follow = true;

        private MechJebModuleFlightRecorder recorder;

        public MechJebModuleFlightRecorderGraph(MechJebCore core)
            : base(core)
        {
            Priority    = 2000;
            graphStates = new graphState[typeCount];
        }

        public override void OnStart(PartModule.StartState state)
        {
            if (HighLogic.LoadedSceneIsEditor)
                return;

            width    = 128 * hSize;
            height   = 128 * vSize;
            recorder = Core.GetComputerModule<MechJebModuleFlightRecorder>();
            ResetScale();
        }

        protected override void WindowGUI(int windowID)
        {
            //DefaultAscentPath path = (DefaultAscentPath)core.GetComputerModule<MechJebModuleAscentAutopilot>().ascentPath;

            if (oldMainBody != MainBody || lastMaximumAltitude != graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].maximum ||
                height != backgroundTexture.height)
            {
                UpdateScale();
                lastMaximumAltitude = graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].maximum;

                if (backgroundTexture == null || height != backgroundTexture.height)
                {
                    Object.Destroy(backgroundTexture);
                    backgroundTexture = new Texture2D(1, height);
                }

                MechJebModuleAscentClassicPathMenu.UpdateAtmoTexture(backgroundTexture, Vessel.mainBody, lastMaximumAltitude, realAtmo);
                oldMainBody = MainBody;
            }

            GUILayout.BeginVertical();

            GUILayout.BeginHorizontal();

            if (GUILayout.Button(paused ? Localizer.Format("#MechJeb_Flightrecord_Button1_1") : Localizer.Format("#MechJeb_Flightrecord_Button1_2"),
                    GUILayout.ExpandWidth(false))) //"Resume""Pause"
            {
                paused = !paused;
            }

            if (GUILayout.Button(
                    downrange ? Localizer.Format("#MechJeb_Flightrecord_Button2_1") : Localizer.Format("#MechJeb_Flightrecord_Button2_2"),
                    GUILayout.ExpandWidth(false))) //"Downrange""Time"
            {
                downrange = !downrange;
            }

            //GUILayout.Label("Size " + (8 * typeCount * recorder.history.Length >> 10).ToString() + "kB", GUILayout.ExpandWidth(false));

            GUILayout.Label(Localizer.Format("#MechJeb_Flightrecord_Label1", GuiUtils.TimeToDHMS(recorder.timeSinceMark)),
                GUILayout.ExpandWidth(false)); //Time <<1>>

            GUILayout.Label(Localizer.Format("#MechJeb_Flightrecord_Label2", recorder.history[recorder.historyIdx].downRange.ToSI()) + "m",
                GUILayout.ExpandWidth(false)); //Downrange <<1>>

            //GUILayout.Label("", GUILayout.ExpandWidth(true));
            GUILayout.FlexibleSpace();

            if (GUILayout.Button(Localizer.Format("#MechJeb_Flightrecord_Button3"), GUILayout.ExpandWidth(false))) //"Mark"
            {
                ResetScale(); // TODO : should check something else to catch Mark calls from other code
                recorder.Mark();
            }

            if (GUILayout.Button(Localizer.Format("#MechJeb_Flightrecord_Button4"), GUILayout.ExpandWidth(false))) //"Reset Scale"
            {
                ResetScale();
            }

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();

            autoScale = GUILayout.Toggle(autoScale, Localizer.Format("#MechJeb_Flightrecord_checkbox1"), GUILayout.ExpandWidth(false)); //Auto Scale

            if (!autoScale && GUILayout.Button("-", GUILayout.ExpandWidth(false)))
            {
                if (downrange)
                    downrangeScale--;
                else
                    timeScale--;
            }

            float maxX = (float)(downrange
                ? recorder.maximums[(int)MechJebModuleFlightRecorder.recordType.DownRange]
                : recorder.maximums[(int)MechJebModuleFlightRecorder.recordType.TimeSinceMark]);

            double maxXScaled = (downrange ? maxX : maxX / precision) / width;
            double autoScaleX = Math.Max(Math.Ceiling(Math.Log(maxXScaled, 2)), 0);
            double manualScaleX = downrange ? downrangeScale : timeScale;
            double activeScaleX = autoScale ? autoScaleX : manualScaleX;

            double scaleX = downrange ? Math.Pow(2, activeScaleX) : precision * Math.Pow(2, activeScaleX);

            GUILayout.Label(downrange ? scaleX.ToSI(2) + "m/px" : GuiUtils.TimeToDHMS(scaleX, 1) + "/px", GUILayout.ExpandWidth(false));

            if (!autoScale && GUILayout.Button("+", GUILayout.ExpandWidth(false)))
            {
                if (downrange)
                    downrangeScale++;
                else
                    timeScale++;
            }

            if (GUILayout.Button("-", GUILayout.ExpandWidth(false)))
            {
                hSize--;
            }

            GUILayout.Label(width.ToString(), GUILayout.ExpandWidth(false));

            if (GUILayout.Button("+", GUILayout.ExpandWidth(false)))
            {
                hSize++;
            }

            GUILayout.Label("x", GUILayout.ExpandWidth(false));

            if (GUILayout.Button("-", GUILayout.ExpandWidth(false)))
            {
                vSize--;
            }

            GUILayout.Label(height.ToString(), GUILayout.ExpandWidth(false));

            if (GUILayout.Button("+", GUILayout.ExpandWidth(false)))
            {
                vSize++;
            }

            timeScale      = Mathf.Clamp(timeScale, 0, 20);
            downrangeScale = Mathf.Clamp(downrangeScale, 0, 20);
            hSize          = Mathf.Clamp(hSize, 1, 20);
            vSize          = Mathf.Clamp(vSize, 1, 10);

            bool oldRealAtmo = realAtmo;

            realAtmo = GUILayout.Toggle(realAtmo, Localizer.Format("#MechJeb_Flightrecord_checkbox2"), GUILayout.ExpandWidth(false)); //Real Atmo

            if (oldRealAtmo != realAtmo)
                MechJebModuleAscentClassicPathMenu.UpdateAtmoTexture(backgroundTexture, Vessel.mainBody, lastMaximumAltitude, realAtmo);

            //GUILayout.Label("", GUILayout.ExpandWidth(true));
            GUILayout.FlexibleSpace();

            if (GUILayout.Button(Localizer.Format("#MechJeb_Flightrecord_Button5"), GUILayout.ExpandWidth(false))) //CSV
            {
                recorder.DumpCSV();
            }

            GUILayout.Label(
                Localizer.Format("#MechJeb_Flightrecord_Label3", (100 * recorder.historyIdx / (float)recorder.history.Length).ToString("F1")),
                GUILayout.ExpandWidth(false)); //Storage: <<1>> %

            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();

            Color color = GUI.color;

            // Blue, Navy, Teal, Magenta, Purple all have poor contrast on black or against other colors here
            // Maybe some of them could be lightened up, but many of the lighter variants are already in this list.

            //ascentPath = GUILayout.Toggle(ascentPath, "Ascent path", GUILayout.ExpandWidth(false));
            stages = GUILayout.Toggle(stages, Localizer.Format("#MechJeb_Flightrecord_checkbox3"), GUILayout.ExpandWidth(false)); //"Stages"

            GUI.color = XKCDColors.White;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].display, Localizer.Format("#MechJeb_Flightrecord_checkbox4"),
                GUILayout.ExpandWidth(false)); //"Altitude"

            GUI.color = XKCDColors.Grey;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeTrue].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeTrue].display, Localizer.Format("#MechJeb_Flightrecord_checkbox5"),
                GUILayout.ExpandWidth(false)); //"True Altitude"

            GUI.color = XKCDColors.LightRed;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Acceleration].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.Acceleration].display, Localizer.Format("#MechJeb_Flightrecord_checkbox6"),
                GUILayout.ExpandWidth(false)); //"Acceleration"

            GUI.color = XKCDColors.Yellow;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedSurface].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedSurface].display, Localizer.Format("#MechJeb_Flightrecord_checkbox7"),
                GUILayout.ExpandWidth(false)); //"Surface speed"

            GUI.color = XKCDColors.Apricot;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedOrbital].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedOrbital].display, Localizer.Format("#MechJeb_Flightrecord_checkbox8"),
                GUILayout.ExpandWidth(false)); //"Orbital speed"

            GUI.color = XKCDColors.Pink;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Mass].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.Mass].display, Localizer.Format("#MechJeb_Flightrecord_checkbox9"),
                GUILayout.ExpandWidth(false)); //"Mass"

            GUILayout.EndHorizontal();
            GUILayout.BeginHorizontal();

            GUI.color = XKCDColors.Cyan;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Q].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.Q].display, Localizer.Format("#MechJeb_Flightrecord_checkbox10"),
                GUILayout.ExpandWidth(false)); //"Q"

            GUI.color = XKCDColors.Lavender;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoA].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.AoA].display, Localizer.Format("#MechJeb_Flightrecord_checkbox11"),
                GUILayout.ExpandWidth(false)); //"AoA"

            GUI.color = XKCDColors.Lime;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoS].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.AoS].display, Localizer.Format("#MechJeb_Flightrecord_checkbox12"),
                GUILayout.ExpandWidth(false)); //"AoS"

            GUI.color = XKCDColors.Orange;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoD].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.AoD].display, Localizer.Format("#MechJeb_Flightrecord_checkbox13"),
                GUILayout.ExpandWidth(false)); //"AoD"

            GUI.color = XKCDColors.Mint;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Pitch].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.Pitch].display, Localizer.Format("#MechJeb_Flightrecord_checkbox14"),
                GUILayout.ExpandWidth(false)); //"Pitch"

            GUI.color = XKCDColors.Beige;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DeltaVExpended].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.DeltaVExpended].display, "∆V", GUILayout.ExpandWidth(false));

            GUI.color = XKCDColors.Green;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.GravityLosses].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.GravityLosses].display, Localizer.Format("#MechJeb_Flightrecord_checkbox15"),
                GUILayout.ExpandWidth(false)); //"Gravity Loss"

            GUI.color = XKCDColors.LightBrown;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DragLosses].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.DragLosses].display, Localizer.Format("#MechJeb_Flightrecord_checkbox16"),
                GUILayout.ExpandWidth(false)); //"Drag Loss"

            GUI.color = XKCDColors.Cerise;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SteeringLosses].display = GUILayout.Toggle(
                graphStates[(int)MechJebModuleFlightRecorder.recordType.SteeringLosses].display, Localizer.Format("#MechJeb_Flightrecord_checkbox17"),
                GUILayout.ExpandWidth(false)); //"Steering Loss"

            GUI.color = color;

            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            GUILayout.BeginHorizontal();

            GUILayout.Space(50);

            GUILayout.Box(Texture2D.blackTexture, GUILayout.Width(width), GUILayout.Height(height));
            Rect r = GUILayoutUtility.GetLastRect();

            DrawScaleLabels(r);

            GUILayout.BeginVertical();

            //GUILayout.Label("X " + hPos.ToSI(2) + " " + (hPos+scaleX*width).ToSI(2), GUILayout.Width(110));

            color = GUI.color;

            if (!graphStates[scaleIdx].display)
            {
                int newIdx = 0;
                while (newIdx < typeCount && !graphStates[newIdx].display)
                {
                    newIdx++;
                }

                if (newIdx == typeCount)
                    newIdx = 0;
                scaleIdx = newIdx;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].display)
            {
                GUI.color = XKCDColors.White;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.AltitudeASL, "ASL " + MuUtils.ToSI(graphStates[(int)recordType.AltitudeASL].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.AltitudeASL].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.AltitudeASL,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox18"), GUILayout.ExpandWidth(true))) //"ASL"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.AltitudeASL;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeTrue].display)
            {
                GUI.color = XKCDColors.Grey;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.AltitudeTrue, "AGL " + MuUtils.ToSI(graphStates[(int)recordType.AltitudeTrue].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.AltitudeTrue].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.AltitudeTrue,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox19"), GUILayout.ExpandWidth(true))) //"AGL"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.AltitudeTrue;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Acceleration].display)
            {
                GUI.color = XKCDColors.LightRed;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.Acceleration, "Acc " + MuUtils.ToSI(graphStates[(int)recordType.Acceleration].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.Acceleration].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.Acceleration,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox20"), GUILayout.ExpandWidth(true))) // "Acc"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.Acceleration;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedSurface].display)
            {
                GUI.color = XKCDColors.Yellow;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.SpeedSurface, "SrfVel " + MuUtils.ToSI(graphStates[(int)recordType.SpeedSurface].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.SpeedSurface].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.SpeedSurface,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox21"), GUILayout.ExpandWidth(true))) //"SrfVel"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.SpeedSurface;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedOrbital].display)
            {
                GUI.color = XKCDColors.Apricot;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.SpeedOrbital, "ObtVel " + MuUtils.ToSI(graphStates[(int)recordType.SpeedOrbital].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.SpeedOrbital].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.SpeedOrbital,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox22"), GUILayout.ExpandWidth(true))) //"ObtVel"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.SpeedOrbital;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Mass].display)
            {
                GUI.color = XKCDColors.Pink;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.Mass, "Mass " + MuUtils.ToSI(graphStates[(int)recordType.Mass].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.Mass].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.Mass,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox23"), GUILayout.ExpandWidth(true))) //"Mass"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.Mass;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Q].display)
            {
                GUI.color = XKCDColors.Cyan;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.Q, "Q " + MuUtils.ToSI(graphStates[(int)recordType.Q].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.Q].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.Q, Localizer.Format("#MechJeb_Flightrecord_checkbox24"),
                        GUILayout.ExpandWidth(true))) //"Q"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.Q;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AoA].display)
            {
                GUI.color = XKCDColors.Lavender;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.AoA, "AoA " + MuUtils.ToSI(graphStates[(int)recordType.AoA].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.AoA].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.AoA,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox25"), GUILayout.ExpandWidth(true))) //"AoA"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.AoA;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AoS].display)
            {
                GUI.color = XKCDColors.Lime;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.AoS, "AoS " + MuUtils.ToSI(graphStates[(int)recordType.AoS].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.AoS].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.AoS,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox26"), GUILayout.ExpandWidth(true))) //"AoS"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.AoS;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AoD].display)
            {
                GUI.color = XKCDColors.Orange;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.AoD, "AoD " + MuUtils.ToSI(graphStates[(int)recordType.AoD].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.AoD].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.AoD,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox27"), GUILayout.ExpandWidth(true))) //"AoD"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.AoD;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Pitch].display)
            {
                GUI.color = XKCDColors.Mint;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.Pitch, "Pitch " + MuUtils.ToSI(graphStates[(int)recordType.Pitch].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.Pitch].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.Pitch,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox28"), GUILayout.ExpandWidth(true))) //"Pitch"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.Pitch;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.DeltaVExpended].display)
            {
                GUI.color = XKCDColors.Beige;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.DeltaVExpended, "DeltaVExpended " + MuUtils.ToSI(graphStates[(int)recordType.DeltaVExpended].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.DeltaVExpended].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.DeltaVExpended, "∆V", GUILayout.ExpandWidth(true)))
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.DeltaVExpended;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.GravityLosses].display)
            {
                GUI.color = XKCDColors.Green;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.GravityLosses, "GravityLosses " + MuUtils.ToSI(graphStates[(int)recordType.GravityLosses].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.GravityLosses].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.GravityLosses,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox29"), GUILayout.ExpandWidth(true))) //"Gravity Loss"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.GravityLosses;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.DragLosses].display)
            {
                GUI.color = XKCDColors.LightBrown;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.DragLosses, "DragLosses " + MuUtils.ToSI(graphStates[(int)recordType.DragLosses].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.DragLosses].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.DragLosses,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox30"), GUILayout.ExpandWidth(true))) //"Drag Loss"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.DragLosses;
            }

            if (graphStates[(int)MechJebModuleFlightRecorder.recordType.SteeringLosses].display)
            {
                GUI.color = XKCDColors.Cerise;
                //if (GUILayout.Toggle(scaleIdx == (int)recordType.SteeringLosses, "SteeringLosses " + MuUtils.ToSI(graphStates[(int)recordType.SteeringLosses].minimum, -1, 3) + " " + MuUtils.ToSI(graphStates[(int)recordType.SteeringLosses].maximum, -1, 3), GUILayout.ExpandWidth(true)))
                if (GUILayout.Toggle(scaleIdx == (int)MechJebModuleFlightRecorder.recordType.SteeringLosses,
                        Localizer.Format("#MechJeb_Flightrecord_checkbox31"), GUILayout.ExpandWidth(true))) //"Steering Loss"
                    scaleIdx = (int)MechJebModuleFlightRecorder.recordType.SteeringLosses;
            }

            GUI.color = color;

            GUILayout.EndVertical();

            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            GUILayout.BeginHorizontal();

            float visibleX = (float)(width * scaleX);
            float rightValue = Mathf.Max(visibleX, maxX);

            if (follow)
                hPos = rightValue - visibleX;
            hPos   = GUILayout.HorizontalScrollbar(hPos, visibleX, 0, rightValue);
            follow = GUILayout.Toggle(follow, "", GUILayout.ExpandWidth(false));

            GUILayout.EndHorizontal();

            if (Event.current.type == EventType.Repaint)
            {
                UpdateScale();

                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].display ||
                    graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeTrue].display)
                    GUI.DrawTexture(r, backgroundTexture, ScaleMode.StretchToFill);

                if (stages)
                    DrawnStages(r, scaleX, downrange);

                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.AltitudeASL, hPos, scaleX, downrange, XKCDColors.White);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeTrue].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.AltitudeTrue, hPos, scaleX, downrange, XKCDColors.Grey);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Acceleration].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.Acceleration, hPos, scaleX, downrange, XKCDColors.LightRed);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedSurface].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.SpeedSurface, hPos, scaleX, downrange, XKCDColors.Yellow);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedOrbital].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.SpeedOrbital, hPos, scaleX, downrange, XKCDColors.Apricot);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Mass].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.Mass, hPos, scaleX, downrange, XKCDColors.Pink);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Q].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.Q, hPos, scaleX, downrange, XKCDColors.Cyan);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AoA].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.AoA, hPos, scaleX, downrange, XKCDColors.Lavender);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AoS].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.AoS, hPos, scaleX, downrange, XKCDColors.Lime);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.AoD].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.AoD, hPos, scaleX, downrange, XKCDColors.Orange);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.Pitch].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.Pitch, hPos, scaleX, downrange, XKCDColors.Mint);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.DeltaVExpended].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.DeltaVExpended, hPos, scaleX, downrange, XKCDColors.Beige);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.GravityLosses].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.GravityLosses, hPos, scaleX, downrange, XKCDColors.Green);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.DragLosses].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.DragLosses, hPos, scaleX, downrange, XKCDColors.LightBrown);
                if (graphStates[(int)MechJebModuleFlightRecorder.recordType.SteeringLosses].display)
                    DrawnPath(r, MechJebModuleFlightRecorder.recordType.SteeringLosses, hPos, scaleX, downrange, XKCDColors.Cerise);

                // Fix : the scales are different so the result is not useful
                //if (ascentPath)
                //    MechJebModuleAscentPathEditor.DrawnPath(r, (float)hScale, (float)graphStates[(int)recordType.AltitudeASL].scale, path, Color.gray);

                width  = 128 * hSize;
                height = 128 * vSize;
            }

            GUILayout.EndVertical();

            base.WindowGUI(windowID);
        }

        private void DrawScaleLabels(Rect r)
        {
            if (scaleIdx == 0)
                return;

            const int w = 80;
            const int h = 20;

            graphState state = graphStates[scaleIdx];
            if (state.labels == null)
                return;

            int count = state.labelsActive;
            double invScaleY = height / (state.maximum - state.minimum);
            float yBase = r.yMax + (float)(state.minimum * invScaleY);
            for (int i = 0; i < count; i++)
            {
                GUI.Label(new Rect(r.xMin - w, yBase - (float)(invScaleY * state.labelsPos[i]) - h * 0.5f, w, h), state.labels[i],
                    GuiUtils.middleRightLabel);
            }
        }

        private void DrawnPath(Rect r, MechJebModuleFlightRecorder.recordType type, float minimum, double scaleX, bool downRange, Color color)
        {
            if (recorder.history.Length <= 2 || recorder.historyIdx == 0)
                return;

            graphState graphState = graphStates[(int)type];

            double scaleY = (graphState.maximum - graphState.minimum) / height;

            double invScaleX = 1 / scaleX;
            double invScaleY = 1 / scaleY;

            float xBase = (float)(r.xMin - minimum * invScaleX);
            float yBase = r.yMax + (float)(graphState.minimum * invScaleY);

            int t = 0;
            while (t < recorder.historyIdx && t < recorder.history.Length &&
                   xBase + (float)((downRange ? recorder.history[t].downRange : recorder.history[t].timeSinceMark) * invScaleX) <= r.xMin)
            {
                t++;
            }

            var p1 = new Vector2(xBase + (float)((downRange ? recorder.history[t].downRange : recorder.history[t].timeSinceMark) * invScaleX),
                yBase - (float)(recorder.history[t][type] * invScaleY));
            var p2 = new Vector2();

            while (t <= recorder.historyIdx && t < recorder.history.Length)
            {
                MechJebModuleFlightRecorder.record rec = recorder.history[t];
                p2.x = xBase + (float)((downRange ? rec.downRange : rec.timeSinceMark) * invScaleX);
                p2.y = yBase - (float)(rec[type] * invScaleY);

                // skip 0 length line but always drawn the first 2 points
                if (r.Contains(p2) && ((p1 - p2).sqrMagnitude >= 1.0 || t < 2))
                {
                    Drawing.DrawLine(p1, p2, color, 2, true);
                    p1.x = p2.x;
                    p1.y = p2.y;
                }

                t++;
            }
        }

        private void DrawnStages(Rect r, double scaleX, bool downRange)
        {
            if (recorder.history.Length <= 2 || recorder.historyIdx == 0)
                return;

            int lastStage = recorder.history[0].currentStage;

            var p1 = new Vector2(0, r.yMin);
            var p2 = new Vector2(0, r.yMax);

            int t = 1;
            while (t <= recorder.historyIdx && t < recorder.history.Length)
            {
                MechJebModuleFlightRecorder.record rec = recorder.history[t];
                if (rec.currentStage != lastStage)
                {
                    lastStage = rec.currentStage;
                    p1.x      = r.xMin + (float)((downRange ? rec.downRange : rec.timeSinceMark) / scaleX);
                    p2.x      = p1.x;

                    if (r.Contains(p1))
                    {
                        Drawing.DrawLine(p1, p2, new Color(0.5f, 0.5f, 0.5f), 1, false);
                    }
                }

                t++;
            }
        }

        private void UpdateScale()
        {
            if (recorder.historyIdx == 0)
                ResetScale();

            for (int t = 0; t < typeCount; t++)
            {
                bool change = false;

                if (graphStates[t].maximum < recorder.maximums[t])
                {
                    change                 = true;
                    graphStates[t].maximum = recorder.maximums[t] + Math.Abs(recorder.maximums[t] * 0.2);
                }

                if (graphStates[t].minimum > recorder.minimums[t])
                {
                    change                 = true;
                    graphStates[t].minimum = recorder.minimums[t] - Math.Abs(recorder.minimums[t] * 0.2);
                }

                if (graphStates[t].labels == null)
                {
                    change                   = true;
                    graphStates[t].labels    = new string[ScaleTicks];
                    graphStates[t].labelsPos = new double[ScaleTicks];
                }

                if (change)
                {
                    double maximum = graphStates[t].maximum;
                    double minimum = graphStates[t].minimum;
                    double range = heckbertNiceNum(maximum - minimum, false);
                    double step = heckbertNiceNum(range / (ScaleTicks - 1), true);

                    minimum = Math.Floor(minimum / step) * step;
                    maximum = Math.Ceiling(maximum / step) * step;
                    int digit = (int)Math.Max(-Math.Floor(Math.Log10(step)), 0);

                    double currX = minimum;
                    int i = 0;
                    while (currX <= maximum + 0.5 * step)
                    {
                        graphStates[t].labels[i]    =  currX.ToString("F" + digit);
                        graphStates[t].labelsPos[i] =  currX;
                        currX                       += step;
                        i++;
                    }

                    graphStates[t].labelsActive = i;
                    graphStates[t].minimum      = minimum;
                    graphStates[t].maximum      = maximum;
                }
            }
        }

        private void ResetScale()
        {
            // Avoid min = max and set sane minimums
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].minimum    = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DownRange].minimum      = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Acceleration].minimum   = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedSurface].minimum   = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedOrbital].minimum   = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Mass].minimum           = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Q].minimum              = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoA].minimum            = -5;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoS].minimum            = -5;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoD].minimum            = 0; // is never negative
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeTrue].minimum   = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Pitch].minimum          = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DeltaVExpended].minimum = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.GravityLosses].minimum  = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DragLosses].minimum     = 0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SteeringLosses].minimum = 0;

            graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeASL].maximum =
                MainBody != null && MainBody.atmosphere ? MainBody.RealMaxAtmosphereAltitude() : 10000.0;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DownRange].maximum      = 500;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Acceleration].maximum   = 2;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedSurface].maximum   = 300;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SpeedOrbital].maximum   = 300;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Mass].maximum           = 5;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Q].maximum              = 1000;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoA].maximum            = 5;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoS].maximum            = 5;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AoD].maximum            = 5;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.AltitudeTrue].maximum   = 100;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.Pitch].maximum          = 90;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DeltaVExpended].maximum = 100;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.GravityLosses].maximum  = 100;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.DragLosses].maximum     = 100;
            graphStates[(int)MechJebModuleFlightRecorder.recordType.SteeringLosses].maximum = 100;
        }

        private double heckbertNiceNum(double x, bool round)
        {
            int exp = (int)Math.Log10(x);
            double f = x / Math.Pow(10.0, exp);
            double nf = 1;

            if (round)
            {
                if (f < 1.5)
                    nf = 1;
                else if (f < 3)
                    nf = 2;
                else if (f < 7)
                    nf = 5;
                else
                    nf = 10;
            }
            else
            {
                if (f <= 1)
                    nf = 1;
                else if (f <= 2)
                    nf = 2;
                else if (f <= 5)
                    nf = 5;
                else
                    nf = 10;
            }

            return nf * Math.Pow(10.0, exp);
        }

        public override GUILayoutOption[] WindowOptions()
        {
            return new[] { GUILayout.Width(400), GUILayout.Height(300) };
        }

        public override string GetName()
        {
            return Localizer.Format("#MechJeb_Flightrecord_title"); //"Flight Recorder"
        }

        public override string IconName()
        {
            return "Flight Recorder";
        }
    }
}

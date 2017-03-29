using System;
using UnityEngine;
using KSP.UI.Screens;
using KSP.IO;


namespace StationKeeping
{
/*	static class Configuration{
		public static readonly bool RealSMA;
		public static readonly double Tolerance;
		public static readonly int WindowX;
		public static readonly int WindowY;
	}
*/
	[KSPAddon(KSPAddon.Startup.TrackingStation, false)]
	public class StationKeeping : MonoBehaviour
	{
		Vessel v;
		bool RealSMA;
		double Tolerance;
		double CurrentSMA;
		double CurrentBodySynchronous;
		string TargetString;
		int Exponent;
		double TargetSMA;

		ApplicationLauncherButton GUIButton;
		bool GUIEnabled;
		const int GUIid = 153537;
		Rect WindowRect;

		public void Start(){
			v = null;
			CurrentSMA = -1e6;
			CurrentBodySynchronous = -1e6;
			TargetString = "0";
			Exponent = 6;

			//load config instead of hardcoding these
			PluginConfiguration Config = PluginConfiguration.CreateForType<StationKeeping> ();
			Config.load ();
			RealSMA = Config.GetValue<bool> ("RealSMA", false);
			Tolerance = Config.GetValue<double> ("Tolerance", 0.01);
			double WindowX = Config.GetValue<double> ("WindowX", 500);
			double WindowY = Config.GetValue<double> ("WindowY", 500);

			WindowRect = new Rect ((float)WindowX, (float)WindowY, 200, 125);

			GameEvents.onGUIApplicationLauncherReady.Add (OnGUIAppLauncherReady);
			GameEvents.onPlanetariumTargetChanged.Add(OnMapTargetChange);
		}

		public void OnDestroy(){
			PluginConfiguration Config = PluginConfiguration.CreateForType<StationKeeping> ();
			//Debug.Log ("[StationKeeping] Saving " + WindowRect);
			Config.SetValue("RealSMA", RealSMA);
			Config.SetValue("Tolerance", Tolerance);
			Config.SetValue("WindowX", (double)WindowRect.x);
			Config.SetValue("WindowY", (double)WindowRect.y);
			//Debug.Log ("Saved WindowX: " + Config ["WindowX"]);
			//Debug.Log ("Saved WindowY: " + Config ["WindowY"]);
			Config.save ();

			GameEvents.onGUIApplicationLauncherReady.Remove(OnGUIAppLauncherReady);
			ApplicationLauncher.Instance.RemoveModApplication(GUIButton);
		}

		void OnMapTargetChange(MapObject mapObject)
		{
			if (mapObject == null || mapObject.type != MapObject.ObjectType.Vessel)
				return;

			v = mapObject.vessel;
			CurrentSMA = v.orbit.semiMajorAxis;
			CelestialBody c = mapObject.vessel.mainBody;
			if (c.angularV != 0)
				CurrentBodySynchronous = Math.Pow (Math.Sqrt (c.gravParameter) / Math.Abs (c.angularV), 2.0 / 3.0);
			else
				CurrentBodySynchronous = Double.PositiveInfinity;
			if (!RealSMA) {
				CurrentSMA -= c.Radius;
				CurrentBodySynchronous -= c.Radius;
			}
		}

		bool CheckSMA (double Current, double Target){
			double x = Current / Target;
			if(x > 1+Tolerance || x < 1 - Tolerance)
				return false;
			return true;
		}

		void ParseTargetString(){
			string NumericString = "";
			bool DecimalPoint = false;
			for (int i = 0; i < TargetString.Length; i++) {
				if (TargetString [i] >= '0' && TargetString [i] <= '9')
					NumericString += TargetString [i];
				if (TargetString [i] == '.' && !DecimalPoint) {
					NumericString += TargetString [i];
					DecimalPoint = true;
				}
			}
			TargetString = NumericString;
			if (TargetString == "" || TargetString == ".")
				TargetSMA = 0;
			else
				TargetSMA = double.Parse (TargetString) * Math.Pow(10, Exponent);
		}

		void SetSMA(double Target){
			if (!RealSMA)
				Target += v.mainBody.Radius;

			if (v.situation != Vessel.Situations.ORBITING) {
				ScreenMessages.PostScreenMessage ("Cannot set station: " + v.vesselName + " not in orbit.");
				return;
			}

			// these energies are actually energy per unit mass
			double OrbitE = v.mainBody.gravParameter / 2 / v.orbit.semiMajorAxis;
			double TargetE = v.mainBody.gravParameter / 2 / Target;
			double AdditionalE = OrbitE - TargetE; // remember orbital energy is negative

			Vector3d pos = v.orbit.getRelativePositionAtUT (Planetarium.GetUniversalTime ());
			Vector3d vel = v.orbit.getOrbitalVelocityAtUT (Planetarium.GetUniversalTime ());
			double DeltaV = Math.Sqrt(vel.sqrMagnitude + 2*AdditionalE) - vel.magnitude;
			//double DeltaV = AdditionalE / vel.magnitude; // approximation

			if (!ConsumeFuel (Math.Abs(DeltaV))) {
				//ScreenMessages.PostScreenMessage ("Cannot set station: " + v.vesselName + " has insufficient fuel.");
				return;
			}

			vel *= 1 + DeltaV / vel.magnitude;
			v.orbit.UpdateFromStateVectors (pos, vel, v.mainBody, Planetarium.GetUniversalTime ());
			v.orbit.semiMajorAxis = Target;

			if (!RealSMA)
				Target -= v.mainBody.Radius;
			ScreenMessages.PostScreenMessage ("Setting orbit of " + v.vesselName + " to " + FormatLength (Target) + ".");

			OnMapTargetChange (v.mapObject);
		}

		/*this SetSMA works, but is unphysical
			//double mna = v.orbit.meanAnomaly;
			v.orbit.semiMajorAxis = Target;
			//v.orbit.meanAnomalyAtEpoch = mna;
			//v.orbit.epoch = Planetarium.GetUniversalTime();
			//-------------------------------

			if (!RealSMA)
				Target -= v.mainBody.Radius;
			ScreenMessages.PostScreenMessage ("Setting orbit of " + v.vesselName + " to " + FormatLength(Target) + ".");

			OnMapTargetChange (v.mapObject);*/

		bool ConsumeFuel(double DeltaV){
			//find best engine
			ModuleEngines Engine = null;
			double isp = -1;
			foreach (ProtoPartSnapshot pp in v.protoVessel.protoPartSnapshots) {
				Part part = PartLoader.getPartInfoByName(pp.partName).partPrefab;
				foreach(ModuleEngines e in part.FindModulesImplementing<ModuleEngines>()){
					//Debug.Log ("[StationKeeping] isp: " + e.atmosphereCurve.Evaluate(0));
					if (e.atmosphereCurve.Evaluate (0) > isp){
						Engine = e;
						isp = e.atmosphereCurve.Evaluate (0);
					}
				}
			}

			if (isp < 0){
				ScreenMessages.PostScreenMessage ("Cannot set station: " + v.vesselName + " has no usable engines.");
				return false;
			}

			//subtract fuel
			// TODO: use the rocket equation.  (not a big deal for 1% tolerance)
			double FuelMass = DeltaV / isp / 9.81 * v.GetTotalMass (); // TODO: look up g rather than hardcoding it
			System.Collections.Generic.List<Propellant> Fuels = Engine.propellants;
			bool InsufficientFuel = false;
			foreach (Propellant f in Fuels) {
				double RequiredUnits = 0;
				//double density = PartResourceLibrary.Instance.GetDefinition (f.name.GetHashCode ()).density;
				RequiredUnits = f.ratio * FuelMass / Engine.mixtureDensity;

				ScreenMessages.PostScreenMessage (v.vesselName + " using " + RequiredUnits.ToString("F2") + " " + f.name + ".");

				foreach (ProtoPartSnapshot pp in v.protoVessel.protoPartSnapshots) {
					foreach (ProtoPartResourceSnapshot r in pp.resources) {
						if (f.name == r.resourceName) {
							double Taken = Math.Min (r.amount, RequiredUnits);
							RequiredUnits -= Taken;
							r.amount -= Taken;
						}
					}
				}
				if (RequiredUnits > 1e-10){
					InsufficientFuel = true;
				}
			}
			if(InsufficientFuel){
				ScreenMessages.PostScreenMessage ("Cannot set station: " + v.vesselName + " has insufficient fuel.");
				return false;
			}

			return true;
		}

		void OnGUIAppLauncherReady(){
			Texture2D Icon = new Texture2D (38, 38);
			Icon.LoadImage (System.IO.File.ReadAllBytes ("GameData/StationKeeping/StationKeeping_new.png"));
			GUIButton = ApplicationLauncher.Instance.AddModApplication(
				OnGUIEnabled,
				OnGUIDisabled,
				DummyVoid,
				DummyVoid,
				DummyVoid,
				DummyVoid,
				ApplicationLauncher.AppScenes.TRACKSTATION,
				Icon);
			//Camera c = UIMasterController.Instance.appCanvas.worldCamera;
			//Vector3 screenPos = c.WorldToScreenPoint(GUIButton.transform.position);
			//WindowRect = new Rect (screenPos.x - 300, screenPos.y - 300, 300, 300);
		}

		void OnGUIEnabled (){
			GUIEnabled = true;
		}

		void OnGUIDisabled (){
			GUIEnabled = false;
		}

		void DummyVoid(){
		}

		public void OnGUI(){
			if (!GUIEnabled || HighLogic.LoadedScene != GameScenes.TRACKSTATION)
				return;

			WindowRect = GUILayout.Window(GUIid, WindowRect, ToolbarWindow, "StationKeeping");
		}

		void ToolbarWindow(int id){
			GUILayout.BeginHorizontal();
			GUILayout.Label ("SMA: ");
			if(CurrentSMA>-1e5)
				GUILayout.Label(FormatLength(CurrentSMA));
			else
				GUILayout.Label("N/A");
			GUILayout.EndHorizontal ();

			GUILayout.BeginHorizontal ();
			GUILayout.Label ("Sync: ");
			if(CurrentBodySynchronous>0)
				GUILayout.Label(FormatLength(CurrentBodySynchronous));
			else
				GUILayout.Label("N/A");
			if(CurrentBodySynchronous < 0 || !CheckSMA(CurrentSMA, CurrentBodySynchronous))
				GUI.enabled = false;
			if (GUILayout.Button ("Set Sync", GUILayout.ExpandWidth(false))) {
				SetSMA(CurrentBodySynchronous);
			}
			GUI.enabled = true;
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal ();
			TargetString = GUILayout.TextField (TargetString);
			ParseTargetString ();
			if(!CheckSMA(CurrentSMA, TargetSMA))
				GUI.enabled = false;
			if (GUILayout.Button ("Set SMA", GUILayout.ExpandWidth(false))) {
				SetSMA(TargetSMA);
			}
			GUI.enabled = true;
			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal ();
			if (GUILayout.Toggle (Exponent == 3, "km"))
				Exponent = 3;
			if (GUILayout.Toggle (Exponent == 6, "Mm"))
				Exponent = 6;
			if (GUILayout.Toggle (Exponent == 9, "Gm"))
				Exponent = 9;
			GUILayout.EndHorizontal();

			GUI.DragWindow ();
		}

		static string FormatLength(double x){
			int level = 0;
			while (x > 1000 && level < 4) {
				x /= 1000;
				level += 1;
			}
			string formatted = x.ToString ("G4");
			if (level == 0)
				formatted += " m";
			else if (level == 1)
				formatted += " km";
			else if (level == 2)
				formatted += " Mm";
			else if (level == 3)
				formatted += " Gm";
			else
				formatted += " Tm";
			return formatted;
		}
	}
}


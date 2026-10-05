// RomPose.cs - the angle at which an axis first runs into the machine.
//
//   RomPose.exe <model.rdyn> --pairs <candidate_pairs.csv> --axis RA_q2
//               --origin x,y,z --dir x,y,z          (joint axis, mm, model frame)
//               --bodies body2,body3,...            (what rides on that joint)
//               [--theta 180] [--coarse 15] [--tol 0.5] [--margin 0] [--out <csv>]
//               [--draw]     keep the model visible (slower, and it is what
//                            exhausted the graphics card once already)
//
// Three facts shape this:
//
//   Utility.Measure reads the MODELLING pose, not the pose an analysis ended
//   at, so a joint angle is asked about by turning the geometry there in the
//   model -- ObjectControl.RotateObjectWithScalar -- and reading interference
//   back.  A test costs a few ms; no solver runs at all.
//
//   RotateObjectWithScalar moves BODIES.  Handed a geometry solid it returns
//   in a fraction of the time, without an error, having moved nothing -- and
//   every axis then reads as unlimited.  Turning one solid per pair would have
//   been far cheaper; it is simply not a thing this API does.
//
//   Turning a body costs about 0.7 s per solid on it, so the angle is what is
//   expensive, not the pair.  Every candidate pair is therefore tested at each
//   angle, and the search is over angles: coarse steps out from where the
//   boxes first meet, then a bisection.
//
// Pairs already interfering at the reference pose are designed fits -- a shaft
// in its housing -- and are reported and then ignored.
using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using FunctionBay.RecurDyn.ProcessNet;
using FunctionBay.RecurDyn.ProcessNet.RecurDyn;

class RomPose
{
    const string Model = "HumanoidUpperBody";

    class Pair
    {
        public string Moving, Other;
        public double Onset;
        public IGeometry Gm, Go;
        public bool Resting;
        public bool Tight;      // already inside the keep-out at the rest pose
    }

    // ---- the modal that will not be switched off ----------------------
    // Rotating a body raises "Check Parametric Point", and RecurDyn keeps
    // raising it: once per rotate call, so roughly a hundred times per axis.
    // ShowWarningMessage=false does not cover it, and deleting every
    // parametric point the API exposes (30 of them) does not either.  A modal
    // dialog also stops the application answering COM, so an unattended run
    // just stops dead in front of it.
    //
    // So it gets answered here, and only it: the watcher matches the exact
    // window title, checks the window belongs to RecurDyn, and presses the
    // button with the OK id.  Every press is logged.  OK is the right answer --
    // it breaks parametric connections in a throwaway copy that is never saved,
    // where Cancel would skip the rotation and quietly report no interference.
    [DllImport("user32.dll")] static extern bool EnumWindows(EnumWindowsProc cb, IntPtr p);
    [DllImport("user32.dll", CharSet = CharSet.Unicode)]
    static extern int GetWindowTextW(IntPtr h, System.Text.StringBuilder s, int n);
    [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
    [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
    [DllImport("user32.dll")] static extern IntPtr GetDlgItem(IntPtr h, int id);
    [DllImport("user32.dll")] static extern IntPtr SendMessage(IntPtr h, uint m, IntPtr w, IntPtr l);
    delegate bool EnumWindowsProc(IntPtr h, IntPtr p);

    const string ModalTitle = "Check Parametric Point";
    const int IDOK = 1;
    static volatile bool watching;
    static int dismissed;

    static void Watch()
    {
        var seen = new HashSet<long>();
        while (watching)
        {
            EnumWindows(delegate (IntPtr h, IntPtr p)
            {
                if (!IsWindowVisible(h)) return true;
                var sb = new System.Text.StringBuilder(256);
                GetWindowTextW(h, sb, 256);
                if (sb.ToString() != ModalTitle) return true;

                uint pid;
                GetWindowThreadProcessId(h, out pid);
                try
                {
                    if (Process.GetProcessById((int)pid).ProcessName
                        .IndexOf("RecurDyn", StringComparison.OrdinalIgnoreCase) < 0) return true;
                }
                catch { return true; }

                IntPtr ok = GetDlgItem(h, IDOK);
                if (ok == IntPtr.Zero) return true;
                // WM_COMMAND / BN_CLICKED, which a modal loop acts on.
                SendMessage(h, 0x0111, (IntPtr)IDOK, ok);
                dismissed++;
                if (seen.Add(h.ToInt64()) && dismissed <= 5)
                    Console.WriteLine("  (answered the parametric-point modal)");
                return true;
            }, IntPtr.Zero);
            Thread.Sleep(150);
        }
    }

    static IModelDocument doc;
    static double[] axisOrigin, axisDir;
    static List<IGeneric> bodies = new List<IGeneric>();
    static double atAngle;
    static int moves;
    static double moveSeconds;

    static void Main(string[] args)
    {
        string modelPath = Path.GetFullPath(args[0]);
        string pairsPath = Str(args, "--pairs", null);
        string axis      = Str(args, "--axis", null);
        string bodyList  = Str(args, "--bodies", null);
        string outPath   = Str(args, "--out", null);
        double theta     = Dbl(args, "--theta", 180.0);
        double coarse    = Dbl(args, "--coarse", 15.0);
        double tol       = Dbl(args, "--tol", 0.5);
        double margin    = Dbl(args, "--margin", 0.0);
        bool   hide      = Array.IndexOf(args, "--draw") < 0;   // hidden unless asked
        // A run that was interrupted leaves its copy open and modified, which
        // locks the file against the next run's fresh copy.  This closes it.
        bool   closeOnly = Array.IndexOf(args, "--close-only") >= 0;
        // Report the smallest gaps at one pose instead of hunting for a limit.
        // Answers "what keep-out is even achievable here", which a sweep does
        // not: the sweep says where a margin is first violated, not how much
        // room the machine has at the pose it actually rests in.
        string gapsAt   = Str(args, "--gaps", null);
        bool   allAxes  = Array.IndexOf(args, "--all-pairs") >= 0;
        axisOrigin = Vec(Str(args, "--origin", null));
        axisDir    = Vec(Str(args, "--dir", null));
        if (closeOnly)
        {
            var rd0  = (IRecurDynApp)new RDApplicationClass();
            var app0 = (IApplication)rd0.RecurDynApplication;
            bool warn = app0.Settings.ShowWarningMessage;
            app0.Settings.ShowWarningMessage = false;   // no "save changes?" prompt
            string want = Path.GetFileNameWithoutExtension(modelPath);
            var c0 = app0.ModelDocumentCollection;
            int closed = 0;
            for (int i = c0.Count - 1; i >= 0; i--)
                if (c0[i].Name.Equals(want, StringComparison.OrdinalIgnoreCase))
                {
                    app0.CloseModelDocument(c0[i]);
                    closed++;
                }
            app0.Settings.ShowWarningMessage = warn;
            Console.WriteLine("closed " + closed + " open '" + want + "'");
            return;
        }

        if (pairsPath == null || axis == null || bodyList == null ||
            axisOrigin == null || axisDir == null)
        {
            Console.WriteLine("usage: RomPose <model.rdyn> --pairs <csv> --axis RA_q2 " +
                              "--origin x,y,z --dir x,y,z --bodies body2,body3,... " +
                              "[--theta 180] [--coarse 15] [--tol 0.5] [--margin 0] [--out <csv>]");
            return;
        }
        if (Path.GetFileNameWithoutExtension(modelPath).Equals(Model, StringComparison.OrdinalIgnoreCase))
            throw new InvalidOperationException("pass a copy, not the original");

        string sub = axis.Substring(0, 2) == "RA" ? "RightArm" : "LeftArm";
        List<Pair> pairs = ReadPairs(Path.GetFullPath(pairsPath), axis, theta, allAxes);
        Console.WriteLine("axis " + axis + ": " + pairs.Count + " candidate pairs within " +
                          theta.ToString("F0", CultureInfo.InvariantCulture) + " deg");

        var rd  = (IRecurDynApp)new RDApplicationClass();
        var app = (IApplication)rd.RecurDynApplication;

        string docName = Path.GetFileNameWithoutExtension(modelPath);
        var col = app.ModelDocumentCollection;
        for (int i = col.Count - 1; i >= 0; i--)
            if (col[i].Name.Equals(docName, StringComparison.OrdinalIgnoreCase))
                app.CloseModelDocument(col[i]);
        doc = app.OpenModelDocument(modelPath);
        Console.WriteLine("opened " + doc.Name);
        // RenderMode lives on IApplication, so it is global and another session
        // would inherit whatever we leave behind.  Wireframe while we turn
        // bodies, then put it back exactly as it was.
        // Only when we are actually drawing.  RenderMode is global, and the
        // user may be looking at their own document in the same application.
        RenderMode render0 = app.RenderMode;
        if (!hide) app.RenderMode = RenderMode.WireFrame;

        // Turning a body whose entities carry parametric points raises a modal
        // "Check Parametric Point" dialog -- OK breaks the parametric
        // connections, Cancel silently skips the rotation.  A modal dialog also
        // stops answering COM, so an unattended run just stops dead.  Breaking
        // them is the right answer here (this is a throwaway copy that is never
        // saved), and the warning is suppressed so nobody has to click it.
        // The setting is application-wide, so it goes back afterwards.
        bool warn0 = app.Settings.ShowWarningMessage;
        app.Settings.ShowWarningMessage = false;
        if (app.Settings.ShowWarningMessage)
            Console.WriteLine("  !! ShowWarningMessage refused to go false");

        // Drawing is what killed the first attempt at this: the GUI ran the
        // graphics card out of memory after enough model loads and rotations
        // (NVIDIA OpenGL "out of memory", then the whole application went).
        // Nothing here needs a picture -- the interference query reads the
        // model, not the screen -- so every layer is hidden while we work and
        // put back afterwards.
        var wasVisible = new Dictionary<uint, bool>();
        if (hide)
        {
            var lc = doc.LayerSetting.LayerCollection;
            for (int i = 0; i < lc.Count; i++)
            {
                ILayer L = lc[i];
                wasVisible[L.LayerNumber] = L.Visible;
                try { L.Visible = false; } catch { }
            }
            Console.WriteLine("hid " + wasVisible.Count + " layers while turning bodies");
        }

        foreach (string bn in bodyList.Split(','))
        {
            var g = doc.GetEntityFromFullName(bn + "@" + sub + "@" + Model) as IGeneric;
            if (g == null) throw new InvalidOperationException("body not found: " + bn);
            bodies.Add(g);
        }
        // ShowWarningMessage does not cover the "Check Parametric Point" modal:
        // it still comes up on the first rotate and stops the run dead, because
        // a modal dialog also stops answering COM.  Its OK button breaks every
        // parametric connection, so we do that up front and deliberately -- on
        // a copy that is never saved -- and then there is nothing left to warn
        // about.  Cancel, by contrast, skips the rotation without an error and
        // the whole sweep quietly reads as "no interference".
        int cut = 0, seen = 0;
        var doomed = new List<IGeneric>();
        foreach (ISubSystem ss in AllSubSystems(doc.Model))
        {
            var pcc = ss.ParametricPointConnectorCollection;
            for (int i = 0; i < pcc.Count; i++)
                try { pcc[i].DeleteAll(); cut++; } catch { }

            // The connectors come back empty on this model, so the points
            // themselves are what has to go.  They live on the subsystem and
            // on each body.
            var spc = ss.ParametricPointCollection;
            for (int i = 0; i < spc.Count; i++) { doomed.Add((IGeneric)spc[i]); seen++; }
            var bc = ss.BodyCollection;
            for (int b = 0; b < bc.Count; b++)
            {
                var bpc = bc[b].ParametricPointCollection;
                for (int i = 0; i < bpc.Count; i++) { doomed.Add((IGeneric)bpc[i]); seen++; }
            }
        }
        int gone = 0;
        foreach (IGeneric pp in doomed)
            try { doc.DeleteEntity(pp); gone++; } catch { }
        Console.WriteLine("parametric points: " + seen + " found, " + gone + " deleted, " +
                          cut + " connectors severed");
        Console.Out.Flush();

        Console.WriteLine("turning " + bodies.Count + " bodies about " +
                          "(" + F(axisOrigin[0]) + "," + F(axisOrigin[1]) + "," + F(axisOrigin[2]) + ")");

        // Resolve the geometry once; GetEntityFromFullName is not free.
        int missing = 0;
        foreach (Pair p in pairs)
        {
            p.Gm = doc.GetEntityFromFullName(p.Moving) as IGeometry;
            p.Go = doc.GetEntityFromFullName(p.Other) as IGeometry;
            if (p.Gm == null || p.Go == null) missing++;
        }
        if (missing > 0) Console.WriteLine("  !! " + missing + " pairs have geometry that is not there");

        // What is already touching at rest is a fit, not a limit.
        int resting = 0;
        // Resting contact is judged on real interference, never on the margin:
        // with a 5 mm keep-out, every pair that merely sits within 5 mm at the
        // reference pose would be written off as a designed fit, and a pair
        // that starts 4 mm away and closes as the joint turns would vanish
        // with it -- taking a real limit along.
        foreach (Pair p in pairs)
            if (p.Gm != null && p.Go != null && Touching(p, 0.0))
            {
                p.Resting = true;
                resting++;
                Console.WriteLine("  at rest: " + Short(p.Moving) + " vs " + Short(p.Other) +
                                  " -- designed fit, ignored");
            }
        Console.WriteLine(resting + " resting fits ignored, " +
                          (pairs.Count - resting - missing) + " pairs live");
        Console.Out.Flush();

        // Nothing can touch before the earliest angle at which any live pair's
        // boxes meet, so the scan starts there.
        // Pairs already inside the keep-out at rest cannot honour it anywhere,
        // so they are held to interference only; they are counted for the log.
        int tight = 0;
        if (margin > 0.0)
            foreach (Pair p in pairs)
                if (!p.Resting && p.Gm != null && p.Go != null && Touching(p, margin))
                { p.Tight = true; tight++; }
        if (tight > 0)
            Console.WriteLine(tight + " pairs are already inside the " + margin +
                              " mm keep-out at rest; those are judged on contact alone");

        double start = theta;
        foreach (Pair p in pairs)
            if (!p.Resting && p.Gm != null && p.Go != null)
                start = Math.Min(start, Math.Abs(p.Onset));

        watching = true;
        var watcher = new Thread(Watch);
        watcher.IsBackground = true;
        watcher.Start();

        if (gapsAt != null)
        {
            double at = double.Parse(gapsAt, CultureInfo.InvariantCulture);
            MoveTo(at);
            var gaps = new List<KeyValuePair<double, Pair>>();
            foreach (Pair p in pairs)
            {
                if (p.Resting || p.Gm == null || p.Go == null) continue;
                double d;
                try { d = doc.Utility.Measure.CalculateDistanceWithGeometries(p.Gm, p.Go); }
                catch { continue; }
                gaps.Add(new KeyValuePair<double, Pair>(d, p));
            }
            gaps.Sort(delegate (KeyValuePair<double, Pair> a, KeyValuePair<double, Pair> b)
                      { return a.Key.CompareTo(b.Key); });
            Console.WriteLine("gaps at " + at + " deg, over " + gaps.Count + " live pairs:");
            for (int i = 0; i < Math.Min(15, gaps.Count); i++)
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  {0,8:F3} mm   {1} vs {2}", gaps[i].Key,
                    Short(gaps[i].Value.Moving), Short(gaps[i].Value.Other)));
            if (gaps.Count > 0)
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "minimum gap {0:F3} mm", gaps[0].Key));
            MoveTo(0.0);
            watching = false;
            app.CloseModelDocument(doc);
            Console.WriteLine("closed (not saved)");
            return;
        }

        var csv = new StringBuilder();
        csv.AppendLine("axis,dir,limit_deg,moving_solid,other_solid,angles_tested,seconds");
        var swAll = Stopwatch.StartNew();

        foreach (double sgn in new double[] { 1.0, -1.0 })
        {
            string d = sgn > 0 ? "+" : "-";
            var sw = Stopwatch.StartNew();
            int tested = 0;
            double lo = start, hi = -1.0;
            bool found = false;
            Pair blocker = null;

            for (double q = start; q <= theta + 1e-9; q += coarse)
            {
                Pair b = FirstHit(pairs, sgn * q, margin);
                tested++;
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  {0}{1,7:F1} deg  {2}", d, q, b == null ? "clear"
                        : "BLOCKED by " + Short(b.Moving) + " vs " + Short(b.Other)));
                Console.Out.Flush();
                if (b != null) { hi = q; blocker = b; found = true; break; }
                lo = q;
            }

            if (found)
            {
                while (hi - lo > tol)
                {
                    double mid = (lo + hi) / 2.0;
                    Pair b = FirstHit(pairs, sgn * mid, margin);
                    tested++;
                    if (b != null) { hi = mid; blocker = b; } else lo = mid;
                }
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "  {0} limit {1:F2} deg   {2} vs {3}",
                    d, sgn * lo, Short(blocker.Moving), Short(blocker.Other)));
            }
            else Console.WriteLine("  " + d + " limit: clear through " + theta + " deg");

            csv.Append(axis).Append(',').Append(d).Append(',')
               .Append(found ? F(sgn * lo) : "").Append(',')
               .Append(blocker == null ? "" : Csv(blocker.Moving)).Append(',')
               .Append(blocker == null ? "" : Csv(blocker.Other)).Append(',')
               .Append(tested).Append(',').Append(F(sw.ElapsedMilliseconds / 1000.0)).AppendLine();
            Console.Out.Flush();
        }

        MoveTo(0.0);                       // leave the model as we found it
        watching = false;
        Console.WriteLine("answered the parametric-point modal " + dismissed + " times");
        if (hide)
        {
            var lc = doc.LayerSetting.LayerCollection;
            for (int i = 0; i < lc.Count; i++)
            {
                ILayer L = lc[i];
                if (wasVisible.ContainsKey(L.LayerNumber))
                    try { L.Visible = wasVisible[L.LayerNumber]; } catch { }
            }
        }
        if (!hide) app.RenderMode = render0;
        app.Settings.ShowWarningMessage = warn0;
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
            "{0} body moves, {1:F0} s turning, {2:F0} s total",
            moves, moveSeconds, swAll.ElapsedMilliseconds / 1000.0));

        if (outPath != null)
        {
            File.WriteAllText(Path.GetFullPath(outPath), csv.ToString(), new UTF8Encoding(false));
            Console.WriteLine("wrote " + Path.GetFullPath(outPath));
        }

        // Nothing is saved: the rotations were a question, not an edit.
        app.CloseModelDocument(doc);
        Console.WriteLine("closed (not saved)");
    }

    // ------------------------------------------------------------------
    static List<ISubSystem> AllSubSystems(ISubSystem root)
    {
        var o = new List<ISubSystem>();
        o.Add(root);
        for (int i = 0; i < root.SubSystemCollection.Count; i++)
            o.AddRange(AllSubSystems(root.SubSystemCollection[i]));
        return o;
    }

    static Pair FirstHit(List<Pair> pairs, double deg, double margin)
    {
        MoveTo(deg);
        foreach (Pair p in pairs)
        {
            if (p.Resting || p.Gm == null || p.Go == null) continue;
            // A pair whose boxes do not meet until later cannot touch here.
            if (Math.Abs(p.Onset) > Math.Abs(deg) + 1e-9) continue;
            if (Touching(p, p.Tight ? 0.0 : margin)) return p;
        }
        return null;
    }

    static bool Touching(Pair p, double margin)
    {
        if (margin > 0.0)
            return doc.Utility.Measure.CalculateDistanceWithGeometries(p.Gm, p.Go) <= margin;
        return doc.Utility.Measure.CalculateInterference(p.Gm, p.Go)
               != InterferenceType.InterferenceType_NoInterference;
    }

    static void MoveTo(double deg)
    {
        double d = deg - atAngle;
        if (Math.Abs(d) < 1e-12) return;
        var sw = Stopwatch.StartNew();
        foreach (IGeneric g in bodies)
            doc.Utility.ObjectControl.RotateObjectWithScalar(g, axisOrigin, axisDir, d);
        atAngle = deg;
        moves++;
        moveSeconds += sw.ElapsedMilliseconds / 1000.0;
    }

    static string Short(string fullName)
    {
        int i = fullName.IndexOf('@');
        return i > 0 ? fullName.Substring(0, i) : fullName;
    }

    static List<Pair> ReadPairs(string path, string axis, double theta, bool allAxes)
    {
        string side = axis.Substring(0, 2), j = axis.Substring(axis.Length - 1);
        var best = new Dictionary<string, Pair>();
        foreach (string line in File.ReadAllLines(path, Encoding.UTF8))
        {
            string[] f = line.Split(',');
            if (f.Length < 9 || f[0] != side) continue;
            if (!allAxes && f[1] != j) continue;
            double q = double.Parse(f[8], CultureInfo.InvariantCulture);
            if (Math.Abs(q) > theta) continue;
            // One entry per solid pair, remembered at the earlier of the two
            // directions: the scan runs both ways from the same list.
            string key = f[4] + " " + f[7];
            if (best.ContainsKey(key) && Math.Abs(best[key].Onset) <= Math.Abs(q)) continue;
            best[key] = new Pair { Moving = f[4], Other = f[7], Onset = q };
        }
        var outp = new List<Pair>(best.Values);
        outp.Sort(delegate (Pair a, Pair b) { return Math.Abs(a.Onset).CompareTo(Math.Abs(b.Onset)); });
        return outp;
    }

    static string Csv(string v)
    {
        return v != null && (v.IndexOf(',') >= 0 || v.IndexOf('"') >= 0)
             ? "\"" + v.Replace("\"", "\"\"") + "\"" : v;
    }

    static string F(double v) { return v.ToString("R", CultureInfo.InvariantCulture); }

    static string Str(string[] a, string n, string d)
    {
        int i = Array.IndexOf(a, n);
        return (i < 0 || i + 1 >= a.Length) ? d : a[i + 1];
    }

    static double Dbl(string[] a, string n, double d)
    {
        string s = Str(a, n, null);
        return s == null ? d : double.Parse(s, CultureInfo.InvariantCulture);
    }

    static double[] Vec(string s)
    {
        if (s == null) return null;
        string[] f = s.Split(',');
        var v = new double[f.Length];
        for (int i = 0; i < f.Length; i++) v[i] = double.Parse(f[i], CultureInfo.InvariantCulture);
        return v;
    }
}

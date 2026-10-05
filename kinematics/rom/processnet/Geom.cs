// Geom.cs - inventory of what could ever touch what.
//
//   Geom.exe <model.rdyn> [--csv <out.csv>]
//
// Prints, per subsystem, every body with its solid geometries, and writes a
// machine-readable row per body: the bounding box in the model frame, the box
// in the body's own reference frame, and where that frame sits.  Those three
// are what the ROM prescreen needs -- a body-frame box plus a link transform
// gives a swept AABB at any joint angle without opening RecurDyn again.
//
// Read-only: the document is closed without saving, and if it was already open
// (the GUI holds a lock on it) it is left exactly as it was found.
using System;
using System.Globalization;
using System.Diagnostics;
using System.IO;
using System.Text;
using FunctionBay.RecurDyn.ProcessNet;
using FunctionBay.RecurDyn.ProcessNet.RecurDyn;

class Geom
{
    static StringBuilder csv = new StringBuilder();
    static StringBuilder sol = new StringBuilder();
    static string stlDir, stlExt, stlOnly;
    static int stlCount;

    static void Main(string[] args)
    {
        string csvPath = StrOpt(args, "--csv", null);
        string solPath = StrOpt(args, "--solids-csv", null);
        stlDir = StrOpt(args, "--stl", null);
        stlExt = StrOpt(args, "--stl-ext", ".stl");
        stlOnly = StrOpt(args, "--stl-bodies", null);
        if (stlDir != null) Directory.CreateDirectory(Path.GetFullPath(stlDir));

        var rd  = (IRecurDynApp)new RDApplicationClass();
        var app = (IApplication)rd.RecurDynApplication;

        // RecurDyn keeps the document locked while it is open in the GUI, so a
        // reopen fails; reuse whatever is already loaded under that name.
        // RecurDyn resolves a relative path against its own working folder, not
        // ours, and answers 0x80040301 when it cannot find the file.
        string modelPath = Path.GetFullPath(args[0]);
        string want = Path.GetFileNameWithoutExtension(modelPath);
        IModelDocument doc = null;
        var col = app.ModelDocumentCollection;
        Console.WriteLine("open documents (" + col.Count + "):");
        for (int i = 0; i < col.Count; i++)
        {
            Console.WriteLine("  " + col[i].Name);
            if (col[i].Name.Equals(want, StringComparison.OrdinalIgnoreCase)) doc = col[i];
        }
        bool opened = false;
        if (doc == null) { doc = app.OpenModelDocument(modelPath); opened = true; }
        Console.WriteLine((opened ? "opened " : "reusing ") + doc.Name);

        csv.AppendLine("subsystem,body,mass_kg,nsolid," +
                       "gx1,gy1,gz1,gx2,gy2,gz2," +   // box in the model frame
                       "bx1,by1,bz1,bx2,by2,bz2," +   // box in the body's own frame
                       "ox,oy,oz,ea,eb,ec");          // that frame, in the model frame
        sol.AppendLine("subsystem,body,solid,fullname,stl,x1,y1,z1,x2,y2,z2");
        Walk(doc.Model, "");

        if (solPath != null)
        {
            File.WriteAllText(Path.GetFullPath(solPath), sol.ToString());
            Console.WriteLine("wrote " + Path.GetFullPath(solPath));
        }
        if (csvPath != null)
        {
            File.WriteAllText(Path.GetFullPath(csvPath), csv.ToString());
            Console.WriteLine("wrote " + Path.GetFullPath(csvPath));
        }

        if (opened) { app.CloseModelDocument(doc); Console.WriteLine("closed (no changes saved)"); }
        else Console.WriteLine("left open (was already open; no changes saved)");
    }

    static void Walk(ISubSystem sub, string ind)
    {
        Console.WriteLine(ind + "SUBSYSTEM " + sub.FullName);
        var bc = sub.BodyCollection;
        Console.WriteLine(ind + "  bodies (" + bc.Count + "):");
        for (int i = 0; i < bc.Count; i++)
        {
            IBody b = bc[i];

            double[] g = ModelBox(b);
            double[] l = BodyBox(b);
            double[] f = FrameAt(b);

            int nSolid = 0, nSheet = 0, nShell = 0;
            try { nSolid = b.GeometrySolidCollection.Count; } catch { }
            try { nSheet = b.GeometrySheetCollection.Count; } catch { }
            try { nShell = b.GeometryShellCollection.Count; } catch { }

            Console.WriteLine(ind + "    " + b.Name + "  mass=" +
                              b.Mass.Value.ToString("G6", CultureInfo.InvariantCulture) +
                              "  solids=" + nSolid + " sheets=" + nSheet + " shells=" + nShell);
            Console.WriteLine(ind + "      model box " + Box(g));
            Console.WriteLine(ind + "      body  box " + Box(l) + "  origin " + Box(f));
            for (int s = 0; s < nSolid; s++)
            {
                var gs = b.GeometrySolidCollection[s];
                double[] sb = null;
                try { sb = (double[])gs.GetBoundingBoxWithRefFrame(b.RefFrame); }
                catch { }
                Console.WriteLine(ind + "        solid: " + gs.Name + "  " + Box(sb));
                string stl = stlDir == null ? "" : ExportStl(b, gs);
                sol.Append(Csv(sub.FullName)).Append(',').Append(Csv(b.Name)).Append(',')
                   .Append(Csv(gs.Name)).Append(',').Append(Csv(gs.FullName)).Append(',')
                   .Append(stl);
                for (int k = 0; k < 6; k++) sol.Append(',').Append(sb == null ? "" : F(sb[k]));
                sol.AppendLine();
            }

            csv.Append(sub.FullName).Append(',').Append(b.Name).Append(',')
               .Append(F(b.Mass.Value)).Append(',').Append(nSolid);
            Row(g); Row(l); Row(f);
            csv.AppendLine();
        }

        var ssc = sub.SubSystemCollection;
        for (int i = 0; i < ssc.Count; i++) Walk(ssc[i], ind + "  ");
    }

    // One file per solid, named by an index so the CAD's Korean part names and
    // their punctuation never reach the filesystem; solid_boxes.csv carries the
    // mapping.  The export is what lets the sweep run offline: RecurDyn's gap
    // scope is exact but costs 30 s to several minutes to create, per pair.
    static string ExportStl(IBody b, IGeometrySolid gs)
    {
        if (stlOnly != null && ("," + stlOnly + ",").IndexOf("," + b.Name + ",") < 0) return "";
        // Which formats this build will actually write is not documented, and a
        // refused extension fails silently -- no exception, no file.  Passing a
        // comma list probes them; the first one that lands wins.
        string file = null;
        var sw = Stopwatch.StartNew();
        try
        {
            foreach (string ext in stlExt.Split(','))
            {
                string cand = Path.Combine(Path.GetFullPath(stlDir),
                              string.Format("{0:0000}{1}", stlCount, ext));
                sw.Restart();
                try { b.FileExportGeometry(gs, cand, true); }
                catch (Exception e) { Console.WriteLine("          -> " + ext + ": " + e.Message); continue; }
                Console.WriteLine("          -> " + Path.GetFileName(cand) + "  " +
                                  (sw.ElapsedMilliseconds / 1000.0).ToString("F1") + " s  " +
                                  (File.Exists(cand) ? new FileInfo(cand).Length + " bytes" : "MISSING"));
                if (File.Exists(cand)) { file = cand; break; }
            }
        }
        catch (Exception e)
        {
            Console.WriteLine("          -> export failed: " + e.Message);
            Console.Out.Flush();
            stlCount++;
            return "";
        }
        Console.Out.Flush();
        stlCount++;
        return file != null ? Path.GetFileName(file) : "";
    }

    // Extents in the model frame, at the pose the model is assembled in.
    static double[] ModelBox(IBody b)
    {
        try
        {
            double x1, y1, z1, x2, y2, z2;
            b.GetBoundingBox(out x1, out y1, out z1, out x2, out y2, out z2);
            return new double[] { x1, y1, z1, x2, y2, z2 };
        }
        catch { return null; }
    }

    // Extents in the body's own frame: this one rides along when the joint
    // turns, so it is the box the prescreen can transform itself.
    static double[] BodyBox(IBody b)
    {
        try { return (double[])b.GetBoundingBoxWithRefFrame(b.RefFrame); }
        catch { return null; }
    }

    // Where that body frame sits in the model frame: origin, then Euler angles.
    static double[] FrameAt(IBody b)
    {
        try
        {
            IReferenceFrame rf = b.RefFrame.AtModel();
            double x, y, z, a, c, d;
            EulerAngle type;
            rf.GetOrigin(out x, out y, out z);
            rf.GetEulerAngleDegree(out type, out a, out c, out d);
            return new double[] { x, y, z, a, c, d };
        }
        catch { return null; }
    }

    static void Row(double[] v)
    {
        for (int i = 0; i < 6; i++) csv.Append(',').Append(v == null ? "" : F(v[i]));
    }

    // Solid names carry commas and non-ASCII in this model's CAD import.
    static string Csv(string v)
    {
        if (v == null) return "";
        return v.IndexOf(',') >= 0 || v.IndexOf('"') >= 0
             ? "\"" + v.Replace("\"", "\"\"") + "\"" : v;
    }

    static string F(double v) { return v.ToString("R", CultureInfo.InvariantCulture); }

    static string Box(double[] v)
    {
        if (v == null) return "(unavailable)";
        return string.Format(CultureInfo.InvariantCulture,
            "[{0,8:F1}{1,8:F1}{2,8:F1} ] .. [{3,8:F1}{4,8:F1}{5,8:F1} ]",
            v[0], v[1], v[2], v[3], v[4], v[5]);
    }

    static string StrOpt(string[] args, string name, string fallback)
    {
        int i = Array.IndexOf(args, name);
        if (i < 0 || i + 1 >= args.Length) return fallback;
        return args[i + 1];
    }
}

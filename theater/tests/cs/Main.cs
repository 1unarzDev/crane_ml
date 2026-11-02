using System;
using System.Collections.Generic;
using System.IO;
using Sim.Performance;

// ctraj-check IN OUT: decode IN with the C# codec, print a summary, re-encode it to OUT.
static class CtrajCheck {
    static void Main(string[] args) {
        CraneTraj.Data d = CraneTraj.Decode(File.ReadAllBytes(args[0]));
        Console.WriteLine($"ticks={d.Ticks} block_ticks={d.BlockTicks} hz={d.Hz} columns={d.Columns.Count} events={d.EventLines.Count}");
        var columns = new List<CraneTraj.Column>();
        foreach (CraneTraj.Column c in d.Columns)
            columns.Add(new CraneTraj.Column { Name = c.Name, Scale = c.Scale, Offset = c.Offset, Values = c.Values });
        File.WriteAllBytes(args[1], CraneTraj.Encode(d.HeaderJson, d.Hz, columns, d.EventLines));
    }
}

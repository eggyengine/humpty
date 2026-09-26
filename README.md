# humpty

*humpty* is a physics library written in Zig and for the eggy engine project. 

## add to project
requires zig `0.16.0` (have not tested for other zig versions, however likely works fine. please open a PR to reduce down the minimum version). 

to use this with the zig build system, import as so:
```bash
zig fetch --save git+https://github.com/eggyengine/humpty
```

and then in `build.zig`:
```zig
const humpty = b.dependency("humpty", .{
    .target = target,
    .optimize = optimize,
});

exe.root_module.addImport("humpty", humpty.module("humpty"));
```

and lastly in your library/executable:
```zig
const humpty = @import("humpty");
```

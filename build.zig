const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const mod = b.addModule("humpty", .{ .root_source_file = b.path("src/root.zig"), .target = target, .optimize = optimize });
    addDocs(b, mod, "humpty");

    // --- imports ---
    const emath = b.dependency("eggenvector", .{
        .target = target,
        .optimize = optimize,
    });

    mod.addImport("eggenvector", emath.module("eggenvector"));

    // ---------------

    const mod_tests = b.addTest(.{
        .root_module = mod,
    });
    const run_mod_tests = b.addRunArtifact(mod_tests);

    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_mod_tests.step);
}

/// `zig build docs`: Zig's HTML API docs for `mod` in zig-out/docs, which
/// .github/workflows/docs.yml publishes to GitHub Pages.
fn addDocs(b: *std.Build, mod: *std.Build.Module, name: []const u8) void {
    const docs = b.addObject(.{ .name = name, .root_module = mod });
    const install = b.addInstallDirectory(.{ .source_dir = docs.getEmittedDocs(), .install_dir = .prefix, .install_subdir = "docs" });
    b.step("docs", "Build the API docs into zig-out/docs").dependOn(&install.step);
}

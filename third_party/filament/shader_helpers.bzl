"""Tools to support the generation of generated shader files.
"""

def generate_compiled_shaders(
        name,
        srcs):
    generated_outputs = []
    for src in srcs:
        src_basename = src.split("/")[-1]
        out_basename = src_basename + ".cpp"
        label = src_basename.replace(".", "_")
        out = "%s" % out_basename
        native.genrule(
            name = "generated_%s" % label,
            srcs = [src],
            outs = [out],
            cmd = (
                "SRCFILE=$(location %s);" % src +
                "OUTFILE=$(@D)/%s;" % out +
                "echo namespace filament { >> $$OUTFILE ; " +
                "echo namespace shaders { >> $$OUTFILE ; " +
                "echo extern const char %s[] = R\\\"FILAMENT__\\( >> $$OUTFILE ; " % label +
                "cat $$SRCFILE >> $$OUTFILE ; " +
                "echo \\)FILAMENT__\\\"\\; >> $$OUTFILE ; " +
                "echo }  // namespace shaders >> $$OUTFILE ; " +
                "echo }  // namespace filament >> $$OUTFILE ; "
            ),
        )
        generated_outputs.append(out)
    native.cc_library(name = name, srcs = generated_outputs)

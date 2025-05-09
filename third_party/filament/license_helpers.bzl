"""Tools to support the generation of licenses.inc include files.
(Filament command line tools have an option to print out 3p licenses).
"""

def list_licenses(
        name,
        srcs,
        outs,
        delim = ""):
    native.genrule(
        name = "%s_combined" % name,
        srcs = srcs,
        outs = outs,
        cmd = """
          echo "R\\"%s(" >> "$@"
          for FILESET in $(SRCS); do
            for FILE in $$(find $$FILESET ! -type d); do
              cat "$$FILE" >> "$@"
            done
          done
          echo ")%s\\"", >> "$@"
      """ % (delim, delim),
    )
    native.cc_library(
        name = name,
        textual_hdrs = outs,
    )

def describe_license(
        name):
    srcs = ["//licenses:%s.LICENSE" % name]
    describe_license_explicit(name, srcs)

def describe_license_explicit(
        name,
        srcs):
    out = "%s_license" % name
    genrule_cmd = "\n".join([
        "echo \"License and copyrights for %s:\n\n\" >> \"$@\"" % name,
        "for f in $(SRCS); do",
        "  cat $$f >> $@",
        "done",
        "echo \"L\n\n\" >> \"$@\"",
    ])
    native.genrule(
        name = name,
        srcs = srcs,
        outs = [out],
        cmd = genrule_cmd,
    )

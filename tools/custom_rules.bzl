
# Create header from CMake *.in template
def _header_template_impl(ctx):
    # this generates the output from the template
    ctx.actions.expand_template(
        template = ctx.file.template,
        output = ctx.outputs.out,
        substitutions = ctx.attr.vars,
    )

    return [
            # create a provider which says that this
            # out file should be made available as a header
            CcInfo(compilation_context=cc_common.create_compilation_context(

                # pass out the include path for finding this header
                includes=depset([ctx.outputs.out.dirname]),

                # and the actual header here.
                headers=depset([ctx.outputs.out])
            ))
        ]

header_template = rule(
    implementation = _header_template_impl,
    attrs = {
        "vars": attr.string_dict(),
        "extension": attr.string(default=".hpp"),
        "template": attr.label(
            mandatory = True,
            allow_single_file = True,
        ),
    },
    outputs = {
        "out": "%{name}%{extension}",
    },
    output_to_genfiles = True,
)
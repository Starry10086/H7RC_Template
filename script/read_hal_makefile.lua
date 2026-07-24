function main(target)
    local hal_dir = "bsp/HAL"
    local makefile = path.join(hal_dir, "Makefile")

    if not os.isfile(makefile) then
        local makefiles = os.files(path.join(hal_dir, "*", "Makefile"))
        if #makefiles == 1 then
            makefile = makefiles[1]
            hal_dir = path.directory(makefile)
        else
            raise("cannot find CubeMX Makefile: expected bsp/HAL/Makefile or one Makefile under bsp/HAL/*")
        end
    end

    local file = io.open(makefile, "r")
    local text = file:read("a"):gsub("\r", ""):gsub("\\ *\n", " ") .. "\n"
    file:close()

    for _, name in ipairs({"C_SOURCES", "ASM_SOURCES", "ASMM_SOURCES", "ASMMC_SOURCE"}) do
        local sources = text:match(name .. "%s*=([^\n]+)\n")
        if sources then
            sources:gsub("[^ ]+", function(f)
                target:add("files", path.join(hal_dir, f))
            end)
        end
    end

    local includes = text:match("C_INCLUDES%s*=([^\n]+)\n")
    if includes then
        includes:gsub("-I([^ ]+)", function(f)
            target:add("includedirs", path.join(hal_dir, f))
        end)
    end
end

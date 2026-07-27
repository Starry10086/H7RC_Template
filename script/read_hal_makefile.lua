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

    local added_sources = {}

    local function add_source(source)
        local fullpath = path.normalize(path.join(hal_dir, source))
        if os.isfile(fullpath) and not added_sources[fullpath] then
            target:add("files", fullpath)
            added_sources[fullpath] = true
        end
    end

    for _, name in ipairs({"C_SOURCES", "ASM_SOURCES", "ASMM_SOURCES", "ASMMC_SOURCE"}) do
        local sources = text:match(name .. "%s*=([^\n]+)\n")
        if sources then
            sources:gsub("[^ ]+", function(f)
                add_source(f)
            end)
        end
    end

    -- Some CubeMX Makefile generations omit the common HAL sources even though
    -- their modules remain enabled. Derive those sources from hal_conf.h.
    add_source("Core/Src/system_stm32h7xx.c")

    local hal_conf = path.join(hal_dir, "Core/Inc/stm32h7xx_hal_conf.h")
    if os.isfile(hal_conf) then
        local conf_file = io.open(hal_conf, "r")
        local conf_text = conf_file:read("a"):gsub("\r", "")
        conf_file:close()

        if conf_text:find("\n%s*#define%s+HAL_MODULE_ENABLED") then
            add_source("Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c")
        end

        for module in conf_text:gmatch("\n%s*#define%s+HAL_([A-Z0-9_]+)_MODULE_ENABLED") do
            local basename = "stm32h7xx_hal_" .. module:lower()
            add_source("Drivers/STM32H7xx_HAL_Driver/Src/" .. basename .. ".c")
            add_source("Drivers/STM32H7xx_HAL_Driver/Src/" .. basename .. "_ex.c")
        end
    end

    for _, name in ipairs({"C_DEFS", "AS_DEFS"}) do
        local defines = text:match(name .. "%s*=([^\n]+)\n")
        if defines then
            defines:gsub("%-D([^ ]+)", function(define)
                target:add("defines", define)
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

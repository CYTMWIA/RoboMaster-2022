-- 工程名
set_project("RMCV")

-- 工程版本
set_version("0.0.1")

-- 条件编译
    -- 神经网络
    option("model_runner")
        -- 默认禁用这个选项
        set_default("none")
        set_showmenu(true)
        set_values(
            "none",
            "openvino_yolox",
            "openvino_sjtu",
            "tensorrt"
        )
        after_check(function (option)
            print("model_runner: ", option:get("values"))
        end)
    option_end()

-- 工具链
set_toolchains("@gcc-11")
-- 编译参数
set_languages("cxx20")
set_optimize("-O2")

-- 外部依赖
add_requires("cmake::OpenCV", { system = true })
add_requires("cmake::Ceres", { system = true })
add_requires("fmt", { system = true })
add_requires("eigen3", { system = true })
add_requires("pthread", { system = true })

add_packages(
    "cmake::OpenCV",
    "cmake::Ceres",
    "fmt",
    "eigen3",
    "pthread"
)

-- 所有源代码共享 src/ 路径
add_includedirs("src/")

-- 模块

target("capture")
    -- 目标类型
    set_kind("static")
    -- 第三方依赖
    add_includedirs("3rdparty/", { public=true })
    add_ldflags("-L$(projectdir)/3rdparty/daheng/libs", "-llibgxiapi.so")
    add_ldflags("-L/opt/HuarayTech/MVviewer/lib", "-llibMVSDK.so")
    -- 源代码
    add_files("src/capture/*.cpp")

target("config")
    -- 目标类型
    set_kind("static")
    -- 第三方依赖
    add_includedirs("3rdparty/toml11")
    -- 源代码
    add_files("src/config/*.cpp")

target("communicate")
    -- 目标类型
    set_kind("static")
    -- 第三方依赖
    add_includedirs("3rdparty/serial/include/")
    add_files("3rdparty/serial/src/**.cc")
    -- 源代码
    add_files("src/communicate/*.cpp")
    add_files("src/communicate/*.c")

target("detect")
    -- 目标类型
    set_kind("static")
    -- 条件编译
    before_build(function (target)
        if is_config("model_runner", "none") then
            target:add("defines", "MODEL_RUNNER_NONE", { public=true })
        elseif is_config("model_runner", "openvino_*") then
            target:add("defines", "MODEL_RUNNER_OPENVINO", { public=true })
            if is_config("model_runner", "openvino_yolox") then

            elseif is_config("model_runner", "openvino_sjtu") then

            else
                raise("未知的model_runner", option)
            end
        elseif is_config("model_runner", "tensorrt") then
            target:add("defines", "MODEL_RUNNER_TENSORRT_SJTU", { public=true })
        else

        end
    end)
    -- 源代码
    add_files("src/detect/*.cpp")

target("predict")
    -- 目标类型
    set_kind("static")
    -- 源代码
    add_files("src/predict/*.cpp")

target("threading")
    -- 目标类型
    set_kind("static")
    -- 源代码
    add_files("src/threading/*.cpp")

target("util")
    -- 目标类型
    set_kind("static")
    -- 源代码
    add_files("src/util/*.cpp")

-- 主程序与工具

target("RMCV")
    -- 类型
    set_kind("binary")
    -- 内部依赖
    add_deps(
        "capture",
        "config",
        "communicate",
        "detect",
        "predict",
        "threading",
        "util"
    )
    -- 源代码
    add_files("src/work_thread/*.cpp")
    add_files("src/*.cpp")
    
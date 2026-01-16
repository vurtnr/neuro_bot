fn main() {
    // 这行代码负责把 robot_interfaces 里的 .msg 变成 Rust 代码
    let _ = r2r::MsgGen::new()
        .package("robot_interfaces")
        .generate();
}
from huggingface_hub import hf_hub_download

gguf_path = hf_hub_download(
    repo_id="YingyingWang/Qwen3_base_instruction_q8",
    filename="qwen3_base_instruction_q8.gguf",
    local_dir="./models/qwen3_base_instruction_q8",
    local_dir_use_symlinks=False
)
print("✅ 下载成功，文件保存路径：", gguf_path)
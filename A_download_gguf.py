from huggingface_hub import hf_hub_download

gguf_path = hf_hub_download(
    repo_id="YingyingWang/Qwen3_base_instruction_q4_k_m",
    filename="Qwen3_base_instruction_q4_k_m.gguf",  # 替换成你模型的文件名
    local_dir="models",  # 指定下载目录
    local_dir_use_symlinks=False  # 避免使用符号链接，强制复制文件
)

print("✅ 下载完成，路径为：", gguf_path)

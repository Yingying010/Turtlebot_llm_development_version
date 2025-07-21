from huggingface_hub import snapshot_download

local_dir = snapshot_download(
    repo_id="YingyingWang/Qwen3_base_instructions",
    local_dir="models/Qwen3_base_instructions",  # ✅ 指定本地保存路径
    local_dir_use_symlinks=False  # ✅ 避免符号链接，强制复制真实文件
)

print("✅ 仓库完整下载完成，保存目录：", local_dir)

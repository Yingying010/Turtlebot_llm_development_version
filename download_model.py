from huggingface_hub import snapshot_download

local_dir = snapshot_download(
    repo_id="YingyingWang/Qwen3_base_instruction_q8",
    local_dir="models/Qwen3_base_instruction_q8",
    local_dir_use_symlinks=False
)

print("✅ 仓库下载完成，保存目录：", local_dir)

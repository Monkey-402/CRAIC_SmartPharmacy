# Git 开发提交流程

本文说明在本仓库（[craic](https://gitee.com/Zi_N/craic)）进行日常开发时，从克隆到推送的完整 Git 流程。

## 前置条件

- 已安装 Git（`git --version` 可查看版本）
- 对 Gitee 仓库有读写权限（私有仓库需配置账号或 SSH 密钥）
- 首次使用建议配置身份（仅需一次）：

```bash
git config --global user.name "你的名字"
git config --global user.email "你的邮箱"
```

## 标准流程一览

```text
git clone  →  git checkout -b  →  修改代码  →  git add  →  git commit  →  git push
  克隆仓库      新建功能分支        本地开发        暂存变更        本地提交        推送到远程
```

---

## 1. 克隆仓库（首次参与开发）

```bash
# HTTPS（需输入 Gitee 账号密码或令牌）
git clone https://gitee.com/Zi_N/craic.git

cd craic
```

若已克隆过，只需进入目录并同步最新代码：

```bash
cd craic
git checkout master
git pull origin master
```

---

## 2. 新建功能分支

**不要直接在 `master` 上改代码**，应基于最新 `master` 拉功能分支：

```bash
git checkout master
git pull origin master

# 分支命名建议：feat/功能名、fix/问题描述、docs/文档说明
git checkout -b feat/your-feature-name
```

示例：

```bash
git checkout -b feat/control-node-goal-queue
git checkout -b fix/nav-topic-remap
```

查看当前分支：

```bash
git branch
```

---

## 3. 本地开发与检查

修改代码后，先确认变更范围：

```bash
git status          # 查看哪些文件被修改/新增
git diff            # 查看未暂存的详细差异
git diff --staged   # 查看已暂存、待提交的差异
```

### 不要提交的内容

本仓库 `.gitignore` 已忽略 Catkin 编译产物，例如：

- `nav_sim_ws/build/`、`nav_sim_ws/devel/`
- `control_ws/build/`、`control_ws/devel/`
- `nav_real_ws/build/`、`nav_real_ws/devel/`

提交前用 `git status` 确认未误加入 `build/`、`devel/` 或大体积模型/权重文件。

---

## 4. 暂存变更（git add）

```bash
# 暂存指定文件
git add control_ws/src/move_nav/src/control_node.cpp
git add nav_sim_ws/README.md

# 暂存当前目录下所有已跟踪文件的修改（慎用，先 git status 看清范围）
git add .

# 交互式挑选要暂存的部分（可选）
git add -p
```

取消暂存：

```bash
git restore --staged <文件路径>
```

---

## 5. 本地提交（git commit）

```bash
git commit -m "简短说明本次改动的目的"
```

### 提交信息建议

- 用中文或英文均可，团队内保持一致即可
- 第一行概括「做了什么、为什么」，控制在 50 字左右
- 需要补充细节时，可写多行：

```bash
git commit -m "$(cat <<'EOF'
feat(control): 增加导航目标点队列

- 支持连续下发多个 goal
- 与 nav_sim_ws 联调通过
EOF
)"
```

本仓库历史提交风格示例：`加入docker相关配置`、`增加真实小车启动节点和消息重映射`。

---

## 6. 推送到远程（git push）

首次推送当前分支需建立上游跟踪：

```bash
git push -u origin feat/your-feature-name
```

之后在同一分支上继续提交，只需：

```bash
git push
```

---

## 7. 合并到主分支（可选）

功能验证通过后，在 Gitee 网页发起 **Pull Request / 合并请求**（`feat/xxx` → `master`），由负责人审核合并。

若团队允许本地合并（需有权限且已充分测试）：

```bash
git checkout master
git pull origin master
git merge feat/your-feature-name
git push origin master
```

合并后可删除本地功能分支：

```bash
git branch -d feat/your-feature-name
```

---

## 常用命令速查

| 场景 | 命令 |
|------|------|
| 查看远程地址 | `git remote -v` |
| 拉取当前分支最新提交 | `git pull` |
| 查看提交历史 | `git log --oneline -10` |
| 撤销工作区未暂存修改 | `git restore <文件>` |
| 修改最近一次提交说明（未 push） | `git commit --amend` |
| 暂存当前未完成工作 | `git stash` / `git stash pop` |

---

## 完整示例（复制即用）

将 `feat/my-change` 和提交说明替换为你的实际内容：

```bash
git clone https://gitee.com/Zi_N/craic.git
cd craic

git checkout master
git pull origin master
git checkout -b feat/my-change

# … 编辑代码、编译测试 …

git status
git add control_ws/src/move_nav/src/control_node.cpp
git commit -m "feat(control): 描述你的改动"

git push -u origin feat/my-change
```

然后在 Gitee 上创建合并请求，将 `feat/my-change` 合并进 `master`。

---

## 遇到问题

| 问题 | 处理思路 |
|------|----------|
| `push` 被拒绝（non-fast-forward） | 先 `git pull --rebase origin <分支名>`，解决冲突后再 `git push` |
| 误提交了 `build/` 目录 | `git rm -r --cached <路径>` 后重新 commit，并确认 `.gitignore` |
| 忘记拉分支直接在 master 改 | `git stash` → `git checkout -b feat/xxx` → `git stash pop` |
| 需要 Gitee 令牌 | 设置 → 私人令牌，用令牌代替密码进行 HTTPS 推送 |

如有疑问，以团队约定的分支策略和 Gitee 仓库权限为准。

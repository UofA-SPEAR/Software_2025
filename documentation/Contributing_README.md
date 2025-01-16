# Contribution Guidelines

### Workflow
We use a feature branch workflow where the `main` branch contains only stable, working code. New branches are created based on their specific purpose, as described below.

### Branch Naming Conventions
Branches must follow the format:  
`<author_name>/<branch_type>/<branch_description>`  

**Branch Types:**  
- `feature`  
- `testing`  
- `bugfix`  
- `hotfix`  
- `documentation`  

PRs that don’t follow this naming convention will be rejected.

### Branching Etiquette
- Each branch should focus on a single feature or task. Related changes across layers are acceptable.  
- For unrelated bugs, switch to `main`, create a bugfix branch, and submit a PR before returning to your feature branch.  
- Use descriptive branch names; vague names will be denied.

### Issue Standards
Every issue requires:  
- Labels  
- Priority  
- Difficulty  
- Type  
- Detailed description  

Active issues (moved from TODO) must also have:  
- An assignee  
- An associated branch (if applicable)  
- Start date  
- Sprint number  

**Issue Description Template:**  

**Title:**  
_Short summary of the task._

**Body:**  
- Detailed description  
- **Acceptance Criteria**: Define completion requirements.  

### Commit Standards
- Use imperative mode in commit messages (e.g., "Fix broken validation").  
- Squash unrelated or minor commits via interactive rebase.  

### Copying Code
Always reference non-original code to prevent plagiarism and assist other contributors.  

### Documentation
Always, after implementing a major change in the rover's functionality or operation, create or update the corresponding `.md` file in the documentation folder. This file should explain how to operate the new feature and provide details on what commands to run.

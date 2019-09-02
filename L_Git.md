# Cheat Sheet for Git

## Create a Project

### Via Github
1. Create a new repository on the Github website.
2. `git clone https://...`

### On Local Machine
```
mkdir foo
cd foo
git init
git status
git add readme.txt
# Origin is name of remote
git remote add origin https://...
git pull origin master
```

## Exclude files from Version Control
```
touch .gitignore
touch password.txt
echo "12345" >> password.txt
echo "# The password file." >> .gitignore
echo "password.txt" >> .gitignore
```

## Workflow
1. Create Files: `touch file{1...5}.txt`
2. Stage Changes: `git add .`
3. Commit Changes: `git commit -m "add files 1 to 5"`
4. Checkout Newly Created Branch `git checkout -b test`
5. Create/Edit Files
```
touch file{6...10}.txt
git rm file3.txt
git rm file4.txt
```
6. Stage and Commit Changes
*Staging is already included by `git rm`.*
```
git add .
git commit -m "add new files 6 to 10, remove 3 and 4"
```
7. Merge Branch into Master
```
git checkout master
git merge test
git branch -d test
```

## Communication with Remote
```
# Remember parameters for next push with -u
git push -u origin master
git pull origin master
```

## Miscellaneous
```
git log
# HEAD = most recent commit
git diff HEAD
git diff -staged
git reset 'file name'
git checkout -- 'file name'
git branch my_new_branch
git checkout my_new_branch
# Combine last two with: git checkout -b my_new_branch
git commit --amend -m "updated commit messages"
git stash
'''
Git is more of a snapshot taker, than a program similar to OneDrive or Dropbox. It keeps track of the history. 

If you have never used git before then set up some common variables with the `config` command,

```bash
$ git config --global user.name "Your Name"
$ git config --global user.email "email@example.com"
$ git config --global push.default simple
$ git config --global pull.rebase true
```

# 1. Setup via SSH

The best way to set up git is to use SSH. The easy way to do this, 

1. Open the terminal and type, 

    ```bash
    $ ssh-keygen
    Generating public/private rsa key pair.
    Enter file in which to save the key (/import/kamen/3/z5555555/.ssh/id_rsa):
    Created directory '/import/kamen/3/z5555555/.ssh'.
    Enter passphrase (empty for no passphrase):
    Enter same passphrase again:
    Your identification has been saved in /import/kamen/3/z5555555/.ssh/id_rsa.
    Your public key has been saved in /import/kamen/3/z5555555/.ssh/id_rsa.pub.
    The key fingerprint is:
    b8:02:31:8b:bf:f5:56:fa:b0:1c:36:89:ad:e1:cb:ad z5555555@williams
    The key's randomart image is:
    ...
    ```

2. Hit enter on everything (default options)
3. Type the following command and copy paste all the text (including the ssh-rsa) into a SSH key setup on the Git client (top right corner dropdown -> settings -> 'SSH Keys' on the left side of the page).

    ```bash
    $ cat ~/.ssh/id_rsa.pub
    ssh-rsa AAAAB3NzaC1yc2EAAAABIwAAAQEAyNSzIDylSPAAGLzUXdw359UhO+tlN6wWprSBc9gu6t3IQ1rvHhPoD6wcRXnonY6ytb00GpS4XRFuhCghx2JNVkXFykJYt3XNr1xkPItMmXr/DRIYrtxTs5sn9el3hHZIgELY8jJZpgIo303kgnF0MsB7XpqCzg7Iv6JGkv7aEoYC/MNr07hXE8iQjYIHDMdO9HxGI80GyMqb1hF+RSpQTNvXQvH56juu9VXt5OwJjOqSVa4SfsEICqdn+3k9w8Z4EaD93Eeog3hz0RoTrme8h/sJenXydJ0w9ZOs0By4fjqKFYPsYEs1K6SHma+kPByZM9COgKHZwOZHH1m24HOITQ== z5555555@williams
    ```


4. That's it! Everything is done. 

The git client (Github, Bitbucket, etc.) now knows to trust your computer and you can clone via SSH from now on. 

## 1.2. Cloning

To make a copy of the **remote** repo onto your local machine go ahead and type the following line in your terminal.
You can get the link by going to the repo, clicking on the clone drop down and then the clipboard under "Clone with SSH".

```bash
$ git clone gitlab@gitlab.cse.unsw.EDU.AU:z9600614/VIP-AI4Everyone-Rescue.git
```

**Note**. Clone via SSH — its a lot easier. 

# 2. Pushing Changes to the Repo

Before you can add any of your changes, make sure that you've pulled any new changes made to the repo. If you skip this step, when you add your own changes there will be conflicts.

```bash
$ git pull
```

Now that you've pulled the latest changes you can go ahead and add your own.

**Note.** Make sure to get used to always running a `git pull` command every time you start work on your code. It's very annoying having to deal with conflicts.

It'll be a good idea to get accustomed with the below 3-step workflow as you'll be doing it a lot.

## 2.1. Adding Changes

Once you're ready to add your changes the first step of the workflow is

```bash
$ git add .
```

or for a specific file(s) you've changed

```bash
$ git add <filename1> <filename2> ...
```

Think of this step as just adding the files that you want to push to the repository, so now git knows which files you want to push.
Once a file is added, it is sent to the git staging area. This is where it'll remain before it is committed. 

If you accidentally added the wrong file. You can use

```bash
$ git reset <filename1>
```

to remove that file.

## 2.2. Committing Changes

Now everything that's been added to the staging area needs to be committed. Make sure to add a commit message explaining the details of your change.
Think of committing as taking a snapshot of your current work and giving a description on what has changed.

```bash
$ git commit -m 'commit message goes here'
```

If the `-m` option is not specified the commit will be taken to the vim text editor where you can type a longer commit message.

## 2.3. Pushing Changes

This is the final step. This is the step where git will push all your changes to the repo. To push your changes simply execute the following command.

```bash
$ git push
```

To verify that all changes have been made use

```bash
$ git status
```

If the push was successful, git status will read that everything is up to date. 
Otherwise, any changed files will be shown in red. 

This command should be used frequently to view where in the development history your changes sit.

# 3. Branching

Branching essentially takes the current branch you are on and makes a copy under another name. This is simply done by:

```bash
$ git branch new_branch_name
```

The `master` branch contains the base code and should only be changed by an admin.

In order to view all branches (both local and remote) use the command

```bash
$ git branch -a
```

or

```bash
$ git show-branch
```

Then to select the branch you want to develop on

```bash
$ git checkout -b <branch>
```

## 3.1. Pushing Remote Branch

When a branch is created remotely, it needs to be registered on the remote git server too. The code below will make sure this happens. 

```bash
$ git push -u origin name_of_branch
```

# 4. Merging

Merging branches is used to combine the work done on two different branches and is where git's magic really comes in. Git will compare the changes done on both branches and decide (based on what changes were done to what sections of the file and when) what to keep. Merges are most often done when a feature branch is complete and ready to be integrated with the master branch.

**Note.** It is strongly recommended, both in this course and in general, to always ensure the code on the `master` branch works correctly and is free of bugs. This is not always easy to achieve, but you should endeavour to keep master as *stable* as possible.

Another recommendation is to merge *master* into your branch *before* merging *your branch* into master as this will ensure that any merge into master will go smoothly.

In general, merges are done by:

```bash
$ git merge [target] # Merge the target branch into the current branch
```

However, a merge done from a branch to master should **always** be done via the merge request on gitlab. This is done as good practice is to have a merge request and then have another user look through the changes of code done on your branch. They may add some comments after reviewing your code. They then may close the request if they see a lot of problems that need to be fixed, or they made approve the merge request. Try to aim for 2-3 approvals before merging your branch to master. 

**Note.** A successful merge automatically uses the commits from the source branch. This means that the commits have already been made, you just need to push these to the server (git push)

## 4.1. Merge Conflicts

A merge conflict occurs when more than one person has changed the same file (hence why we always git pull before working). As a result git doesn't know which lines of code to keep and which to discard and thus we need to go through the files and pick which lines we are going to keep and which lines we are going to delete. 

A merge conflict is physically shown in the file in which it occurs. `<<<<<<<` marks the beginning of the conflicting changes made on the **current** (merged into) branch. `=======` marks the beginning of the conflicting changes made on the **target** (merged) branch. `>>>>>>>` marks the end of the conflict zone.

# 5. Merge Requests

In order to send a Merge request you acknowledge your code is ready to be added to the master branch. The actual Merge request itself is created on gitlab and is very straightforward. Remember to be detailed and concise when describing the new changes your code will have.

# 6. Rebase

Whenever the `master` branch is ahead of your own branch use the following command to restore your branch to the current version of `master`.

```bash
$ git rebase master
```

## Using Github
This aims to guide new users on how to use github and to have good git-hygiene.

1) When on the main repository 'AI4Everyone-Rescue' click the green '<> Code' button. Make sure you are in the 'Local/HHTTPS', copy link.
2) Navigate to the directory you want this repository to be in. Go into the terminal for that directory:
```
git clone [copied link]
```
3) Make a new branch/ switching to branches:
```
git checkout -b [branch_name]
```
**Make changes to your code accordingly**

4) Add files to branch:
```
git add [filename]
```
5) Commit changes: Once you have added files onto the branch, you must commit and state your changes
```
git commit -m "commit message"
```
Please make sure to make it concise but not ambiguous eg:
```
git commit -m "implemented victim detection system"
```
6) Push your changes:
```
git push 
```
Congratulations! You have now pushed code into the repository and you should be able to see it on GitHub. It is good practice to not merge accept your own code. Further information on git commands can be found here [LINK](https://www.atlassian.com/git/glossary#commands)
   

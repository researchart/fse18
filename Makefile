typo:  ready 
	@- git status
	@- git commit -am "saving"
	@- git push origin master

commit:  ready 
	@- git status
	@- git commit -a 
	@- git push origin master

update: ready
	@- git pull origin master

status: ready
	@- git status

ready: gitting 

gitting: timm
	@git config --global credential.helper cache
	@git config credential.helper 'cache --timeout=3600'

olga:
	@git config --global user.name "Olga Baysal"
	@git config --global user.email olgabaysal@gmail.com

timm:
	@git config --global user.name "Tim Menzies"
	@git config --global user.email tim.menzies@gmail.com



#Requires -Version 3.0
$srcfiles = @()
$Excludes = (".*", "build", ".git")
Get-ChildItem -Exclude $Excludes | Get-ChildItem -File -Recurse | where {$_.extension -in ".c",".h"} | 
	ForEach {
		$srcfiles += $_.FullName
	}
powershell "clang-format -style=file --verbose -i ${srcfiles}"

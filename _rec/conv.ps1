$root="d:\45 168S1P_Unit_PackBMSR03\F28069PackBMS"
$cp949=[System.Text.Encoding]::GetEncoding(949)
$u=New-Object System.Text.UTF8Encoding($false)
$b=[System.IO.File]::ReadAllBytes("$root\_rec\param_git.bin")
$t=$cp949.GetString($b)
[System.IO.File]::WriteAllText("$root\_rec\param_git_utf8.txt",$t,$u)
$lines=($t -replace "`r`n","`n") -split "`n"
"git lines=$($lines.Count)"
$todos=@($lines | Where-Object {$_ -match 'TODOS'})
"git TODOS count=$($todos.Count)"

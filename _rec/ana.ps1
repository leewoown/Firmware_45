$root="d:\45 168S1P_Unit_PackBMSR03\F28069PackBMS"
$cp949=[System.Text.Encoding]::GetEncoding(949)
$us=New-Object System.Text.UTF8Encoding($false,$true)
$b=[System.IO.File]::ReadAllBytes("$root\_rec\param_git.bin")
"=== git HEAD parameter.h ($($b.Length) bytes) ==="
$utf8ok=$true; try{$tu=$us.GetString($b)}catch{$utf8ok=$false}
"UTF8 valid=$utf8ok"
$tc=$cp949.GetString($b)
function moji($t){ $n=0; foreach($c in $t.ToCharArray()){ $v=[int][char]$c; if($v -eq 0x5360 -or $v -eq 0x5B43 -or $v -eq 0x8ADB){$n++} }; return $n }
"CP949 decode: mojibake=$(moji $tc), hasKorean=$($tc -match '[가-힣]')"
if($utf8ok){ "UTF8 decode: mojibake=$(moji $tu)" }
"--- git(CP949 decode) TODOS 줄 ---"
($tc -replace "`r`n","`n" -split "`n") | Where-Object{$_ -match 'TODOS'} | Select-Object -First 6 | ForEach-Object{ "  " + $_.Trim() }
"--- git(CP949 decode) 일반 한글 주석 줄 ---"
($tc -replace "`r`n","`n" -split "`n") | Where-Object{($_ -match '[가-힣]') -and ($_ -notmatch 'TODOS')} | Select-Object -First 4 | ForEach-Object{ "  " + $_.Trim() }

const fs = require('fs');
const path = 'd:/45 168S1P_Unit_PackBMSR03/F28069PackBMS/SysInclude/parameter.h';
const buf = fs.readFileSync(path);
// UTF-8 validity (strict)
let utf8ok = true;
try { new TextDecoder('utf-8', { fatal: true }).decode(buf); } catch (e) { utf8ok = false; }
const c = buf.toString('utf8');
const lines = c.split(/\r?\n/);
const jeom = [], hanja = [];
for (let i = 0; i < lines.length; i++) {
  const isT = /TODOS/.test(lines[i]) ? '(TODOS)' : '';
  if (/占/.test(lines[i])) jeom.push((i + 1) + isT);          // 占
  if (/[一-鿿]/.test(lines[i]) && !/占/.test(lines[i])) hanja.push((i + 1) + isT); // CJK ideograph (寃利형)
}
console.log('UTF8 valid =', utf8ok);
console.log('占형(비가역) 줄 [' + jeom.length + ']:', jeom.join(','));
console.log('한자(寃利형) 줄 [' + hanja.length + ']:', hanja.join(','));

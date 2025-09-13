# hdl_global_localization

![hdl_global_localization](https://user-images.githubusercontent.com/31344317/105116113-71fc6180-5b0d-11eb-9d85-bbea922dde84.gif)



## what's difference?
- modified slice function GlobalLocalizationBBS::Points2D GlobalLocalizationBBS::slice in src/engines/global_localization_bbs 
- if there's big variance in z values in map pcd, it's hard to perform bbs_localization
- it aims to dynamically project map considering its surrounding z-values distribution 


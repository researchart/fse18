# set working directory (see https://stackoverflow.com/a/35842119)
dir = tryCatch({
  # script being sourced
  getSrcDirectory()[1]
}, error = function(e) {
  # script being run in RStudio
  dirname(rstudioapi::getActiveDocumentContext()$path)
})
setwd(dir)

# load and merge samples
source("load_samples.R")

nrow(sample1)
# 122
nrow(sample2)
# 127
nrow(sample3)
# 86
nrow(merged)
# 335



#### MONITORING ####
monitoring_f <- factor(monitoring, levels=c("1", "2", "3"), labels=c("Yes", "No", "Dont want"), ordered=TRUE)
monitoring_t <- table(monitoring_f) 
monitoring_p <- lapply(monitoring_t, function(x) {
  scales::percent(x/length(na.omit(monitoring_f)))
})
ylim <- c(0, 1.1*max(monitoring_t))
b <- barplot(monitoring_t, ylim=ylim)
text(x=b, y=monitoring_t, label=monitoring_p, pos=3)
monitoring_t
# Yes        No Dont want 
# 79       125         0 
monitoring_p
# $Yes "38.7%"
# $No "61.3%"
# $`Dont want` "0%"

# s2, yes
length(which(sample2$q10_monitoring == 1))
# 45
# s2, no
length(which(sample2$q10_monitoring == 2))
# 76

# s3, yes
length(which(sample3$q10_monitoring == 1))
# 34
# s3, no
length(which(sample3$q10_monitoring == 2))
# 49


#### DECLINE ####
decline_f <- factor(decline, levels=c("1", "2", "3"), labels=c("Yes", "No", "Dont want"), ordered=TRUE)
decline_t <- table(decline_f) 
decline_p <- lapply(decline_t, function(x) {
  scales::percent(x/length(na.omit(decline_f)))
})
ylim <- c(0, 1.1*max(decline_t))
b <- barplot(decline_t, ylim=ylim)
text(x=b, y=decline_t, label=decline_p, pos=3)
decline_t
# Yes        No Dont want 
# 85       120         0 
decline_p
# $Yes "41.5%"
# $No "58.5%"
# $`Dont want` "0%"


#### MENTOR ####
mentor_f <- factor(mentor, levels=c("1", "2", "3"), labels=c("Yes", "No", "Dont want"), ordered=TRUE)
mentor_t <- table(mentor_f) 
mentor_p <- lapply(mentor_t, function(x) {
  scales::percent(x/length(na.omit(mentor_f)))
})
ylim <- c(0, 1.1*max(mentor_t))
b <- barplot(mentor_t, ylim=ylim)
text(x=b, y=mentor_t, label=mentor_p, pos=3)
mentor_t
# Yes        No Dont want 
# 114        96         0
mentor_p
# $Yes "54.3%"
# $No "45.7%"
# $`Dont want` "0%"

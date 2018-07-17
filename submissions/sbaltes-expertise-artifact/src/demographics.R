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

demographics <- data.frame(Sample=as.integer(c(1, 2, 3)),
                           N=as.integer(c(nrow(sample1), nrow(sample2), nrow(sample3))),
                           stringsAsFactors=FALSE)


#### AGE ####

boxplot(list(sample1$q16_age, sample2$q16_age, sample3$q16_age),
        main="Age", names=c("sample 1", "sample 2", "sample 3"))

# http://stackoverflow.com/a/3483388
b <- boxplot(list(sample1$q16_age, sample2$q16_age, sample3$q16_age),
             main="Age",names=c("sample 1", "sample 2", "sample 3"), plot=0)
boxplot(list(sample1$q16_age, sample2$q16_age, sample3$q16_age),
        main="Age", names=paste(b$names, " (n=", b$n, ")", sep=""))

summary(age)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max.    NA's 
#  16.00   26.00   33.00   38.62   55.00   74.00      16 
sd(na.omit(age))
# 14.85812
length(na.omit(age))
# 319
summary(sample1$q16_age)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max.    NA's 
#  17.00   26.00   29.00   30.44   34.00   54.00       6 
sd(na.omit(sample1$q16_age))
# 6.360597
length(na.omit(sample1$q16_age))
# 116
summary(sample2$q16_age)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max.    NA's 
#   16.00   24.00   30.00   31.58   36.00   62.00       8
sd(na.omit(sample2$q16_age))
# 10.00296
length(na.omit(sample2$q16_age))
# 119

summary(sample3$q16_age)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max.    NA's 
#  50.00   56.00   59.00   59.90   62.25   74.00       2 
sd(na.omit(sample3$q16_age))
# 4.942121
length(na.omit(sample3$q16_age))
# 84
table(sample3$q16_age)
# 50 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 71 72 74 
# 1  6 12  4  6  8 11  7  4  4  3  2  2  4  3  1  2  1  2  1

# add age info for demographics table
demographics$AgeMean <- as.numeric(c(mean(sample1$q16_age, na.rm=TRUE), mean(sample2$q16_age, na.rm=TRUE), mean(sample3$q16_age, na.rm=TRUE)))
demographics$AgeSD <- as.numeric(c(sd(sample1$q16_age, na.rm=TRUE), sd(sample2$q16_age, na.rm=TRUE), sd(sample3$q16_age, na.rm=TRUE)))
demographics$AgeMedian <- as.numeric(c(median(sample1$q16_age, na.rm=TRUE), median(sample2$q16_age, na.rm=TRUE), median(sample3$q16_age, na.rm=TRUE)))


#### WORK TIME ####
summary(work_time)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max.    NA's 
#   0.00   50.00   80.00   69.42   90.00  100.00       2 
sd(na.omit(work_time))
# 27.86668
length(na.omit(work_time))
# 333
summary(sample1$q19_work_time)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max.    NA's 
#   0.00   50.00   80.00   70.21   90.00  100.00       1 
sd(na.omit(sample1$q19_work_time))
# 26.34869
length(na.omit(sample1$q19_work_time))
# 121

summary(sample2$q19_work_time)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max.    NA's 
#   0.00   50.00   80.00   69.52   90.00  100.00       1 
sd(na.omit(sample2$q19_work_time))
# 26.4417
length(na.omit(sample2$q19_work_time))
# 126

summary(sample3$q19_work_time)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 0.00   50.00   80.00   68.17   90.00  100.00 
sd(na.omit(sample3$q19_work_time))
# 31.97396
length(na.omit(sample3$q19_work_time))
# 86

# add work time info for demographics table
demographics$WorkTimeMean <- as.numeric(c(mean(sample1$q19_work_time, na.rm=TRUE), mean(sample2$q19_work_time, na.rm=TRUE), mean(sample3$q19_work_time, na.rm=TRUE)))
demographics$WorkTimeSD <- as.numeric(c(sd(sample1$q19_work_time, na.rm=TRUE), sd(sample2$q19_work_time, na.rm=TRUE), sd(sample3$q19_work_time, na.rm=TRUE)))
demographics$WorkTimeMedian <- as.numeric(c(median(sample1$q19_work_time, na.rm=TRUE), median(sample2$q19_work_time, na.rm=TRUE), median(sample3$q19_work_time, na.rm=TRUE)))


#### EXPERIENCE AND EXPERTISE ####

# plot median and mean of experience for all samples 
plot(median(sample1$q8_experience_general), median(sample1$q9_experience_java),
     type="p", pch=17, xlim=c(0,40), ylim=c(0,10))
points(mean(sample1$q8_experience_general), mean(sample1$q9_experience_java), pch=2)
points(median(sample2$q1_experience_general), median(sample2$q2_experience_java), pch=15)
points(mean(sample2$q1_experience_general), mean(sample2$q2_experience_java), pch=0)
points(median(sample3$q1_experience_general), median(sample3$q2_experience_java), pch=16)
points(mean(sample3$q1_experience_general), mean(sample3$q2_experience_java), pch=1)
legend("topright", legend=c("sample1", "sample2", "sample3"), pch=c(17, 15, 16))

# boxplots for experience distribution in samples
par(mfrow=c(1, 2)) 
b <- boxplot(list(sample1$q8_experience_general, sample2$q1_experience_general, sample3$q1_experience_general),
             main="General experience", names=c("S1", "S2", "S3"), plot=FALSE)
boxplot(list(sample1$q8_experience_general, sample2$q1_experience_general, sample3$q1_experience_general),
        main="General experience", names=paste(b$names, " (n=", b$n, ")", sep=""))
b <- boxplot(list(sample1$q9_experience_java, sample2$q2_experience_java, sample3$q2_experience_java),
             main="Java experience", names=c("S1", "S2", "S3"), plot=FALSE)
boxplot(list(sample1$q9_experience_java, sample2$q2_experience_java, sample3$q2_experience_java),
        main="Java experience", names=paste(b$names, " (n=", b$n, ")", sep=""))
par(mfrow=c(1, 1))

# comparision of samples
length(na.omit(sample1$q8_experience_general[sample1$q8_experience_general>20]))
# 6
length(na.omit(sample2$q1_experience_general[sample2$q1_experience_general>20]))
# 22

# sample 1
summary(sample1$q8_experience_general)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 2.00    7.00   10.00   11.84   15.00   35.00
summary(sample1$q9_experience_java)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 0.000   1.000   3.500   4.959   7.000  20.000
summary(sample1$q10_expertise_general)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 2.000   4.000   5.000   4.828   6.000   6.000 
summary(sample1$q11_expertise_java)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.000   3.000   4.000   3.639   5.000   6.000 

# sample 2
summary(sample2$q1_experience_general)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.00    6.00   10.00   12.75   16.00   40.00 
summary(sample2$q2_experience_java)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 0.000   3.000   6.000   7.559  12.000  20.000
summary(sample2$q3_expertise_general)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 2.000   4.000   5.000   4.787   6.000   6.000 
summary(sample2$q4_expertise_java)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.000   4.000   5.000   4.409   6.000   6.000 

# sample 3
summary(sample3$q1_experience_general)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 10.00   30.00   35.00   34.06   40.00   52.00 
summary(sample3$q2_experience_java)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 0.000   0.000   1.500   5.651   8.000  25.000
summary(sample3$q3_expertise_general)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 2.000   5.000   5.000   5.256   6.000   6.000
summary(sample3$q4_expertise_java)
# Min. 1st Qu.  Median    Mean 3rd Qu.    Max. 
# 1.000   1.000   2.000   2.826   4.000   6.000 

# plot median and mean of expertise for all samples
plot(median(sample1$q10_expertise_general), median(sample1$q11_expertise_java),
     type="p", pch=17, xlim=c(1,6), ylim=c(1,6))
points(mean(sample1$q10_expertise_general), mean(sample1$q11_expertise_java), pch=2)
points(median(sample2$q3_expertise_general), median(sample2$q4_expertise_java), pch=15)
points(mean(sample2$q3_expertise_general), mean(sample2$q4_expertise_java), pch=0)
points(median(sample3$q3_expertise_general), median(sample3$q4_expertise_java), pch=16)
points(mean(sample3$q3_expertise_general), mean(sample3$q4_expertise_java), pch=1)
legend("topright", legend=c("sample1", "sample2", "sample3"), pch=c(17, 15, 16))

# boxplots for expertise distribution in samples
par(mfrow=c(1, 2)) 
b <- boxplot(list(sample1$q10_expertise_general, sample2$q3_expertise_general, sample3$q3_expertise_general),
             main="General expertise", names=c("S1", "S2", "S3"), plot=0)
boxplot(list(sample1$q10_expertise_general, sample2$q3_expertise_general, sample3$q3_expertise_general),
        main="General expertise", names=paste(b$names, " (n=", b$n, ")", sep=""))
b <- boxplot(list(sample1$q11_expertise_java, sample2$q4_expertise_java, sample3$q4_expertise_java),
             main="Java expertise", names=c("S1", "S2", "S3"), plot=0)
boxplot(list(sample1$q11_expertise_java, sample2$q4_expertise_java, sample3$q4_expertise_java),
        main="Java expertise", names=paste(b$names, " (n=", b$n, ")", sep=""))
par(mfrow=c(1, 1))


library(likert)
likert_data <- data.frame(
  factor(sample1$q10_expertise_general, levels=c(1:6)),
  factor(sample1$q11_expertise_java, levels=c(1:6))
)
names(likert_data) <- c("expertise general", "expertise java")
plot(likert(likert_data))
detach("package:likert", unload=TRUE)

# add work time info for demographics table
demographics$GeneralExperienceMean <- as.numeric(c(mean(sample1$q8_experience_general, na.rm=TRUE), mean(sample2$q1_experience_general, na.rm=TRUE), mean(sample3$q1_experience_general, na.rm=TRUE)))
demographics$GeneralExperienceMedian <- as.numeric(c(median(sample1$q8_experience_general, na.rm=TRUE), median(sample2$q1_experience_general, na.rm=TRUE), median(sample3$q1_experience_general, na.rm=TRUE)))

demographics$JavaExperienceMean <- as.numeric(c(mean(sample1$q9_experience_java, na.rm=TRUE), mean(sample2$q2_experience_java, na.rm=TRUE), mean(sample3$q2_experience_java, na.rm=TRUE)))
demographics$JavaExperienceMedian <- as.numeric(c(median(sample1$q9_experience_java, na.rm=TRUE), median(sample2$q2_experience_java, na.rm=TRUE), median(sample3$q2_experience_java, na.rm=TRUE)))

demographics$GeneralExpertiseMean <- as.numeric(c(mean(sample1$q10_expertise_general, na.rm=TRUE), mean(sample2$q3_expertise_general, na.rm=TRUE), mean(sample3$q3_expertise_general, na.rm=TRUE)))
demographics$GeneralExpertiseMedian <- as.numeric(c(median(sample1$q10_expertise_general, na.rm=TRUE), median(sample2$q3_expertise_general, na.rm=TRUE), median(sample3$q3_expertise_general, na.rm=TRUE)))

demographics$JavaExpertiseMean <- as.numeric(c(mean(sample1$q11_expertise_java, na.rm=TRUE), mean(sample2$q4_expertise_java, na.rm=TRUE), mean(sample3$q4_expertise_java, na.rm=TRUE)))
demographics$JavaExpertiseMedian <- as.numeric(c(median(sample1$q11_expertise_java, na.rm=TRUE), median(sample2$q4_expertise_java, na.rm=TRUE), median(sample3$q4_expertise_java, na.rm=TRUE)))


#### EXPORT ####

# write data for demographics table
write.table(demographics, file="../results/demographics.csv", sep=",", col.names=TRUE, row.names=FALSE, na="", quote=TRUE, qmethod="double", fileEncoding="UTF-8")

# write boxplots for experience and expertise distributions in samples

pdf("../results/boxplots_experience_expertise.pdf", bg="transparent", width=18, height=4)
par(
  bg="white",
  omi=c(0,0,0,0), # global margins in inches (bottom, left, top, right)
  mai=c(0.5,0.4,0.5,0.4), # subplot margins in inches (bottom, left, top, right)
  mfrow = c(1, 4),
  #pin = (width, height)
  # mfcol # draw in columns
  # increase font size
  cex=1.1,
  cex.main=1.1,
  cex.sub=1,
  cex.lab=1,
  cex.axis=1
)

b <- boxplot(list(sample1$q8_experience_general, sample2$q1_experience_general, sample3$q1_experience_general),
             main="General experience", ylim=c(0,55), names=c("S1", "S2", "S3"), plot=FALSE)
boxplot(list(sample1$q8_experience_general, sample2$q1_experience_general, sample3$q1_experience_general),
        main="General experience", ylim=c(0,55), names=paste(b$names, " (n=", b$n, ")", sep=""))
b <- boxplot(list(sample1$q9_experience_java, sample2$q2_experience_java, sample3$q2_experience_java),
             main="Java experience", ylim=c(0,55), names=c("S1", "S2", "S3"), plot=FALSE)
boxplot(list(sample1$q9_experience_java, sample2$q2_experience_java, sample3$q2_experience_java),
        main="Java experience", ylim=c(0,55), names=paste(b$names, " (n=", b$n, ")", sep=""))
b <- boxplot(list(sample1$q10_expertise_general, sample2$q3_expertise_general, sample3$q3_expertise_general),
             main="General expertise", ylim=c(1,6), names=c("S1", "S2", "S3"), plot=0)
boxplot(list(sample1$q10_expertise_general, sample2$q3_expertise_general, sample3$q3_expertise_general),
        main="General expertise", ylim=c(1,6), names=paste(b$names, " (n=", b$n, ")", sep=""))
b <- boxplot(list(sample1$q11_expertise_java, sample2$q4_expertise_java, sample3$q4_expertise_java),
             main="Java expertise", ylim=c(1,6), names=c("S1", "S2", "S3"), plot=0)
boxplot(list(sample1$q11_expertise_java, sample2$q4_expertise_java, sample3$q4_expertise_java),
        main="Java expertise", ylim=c(1,6), names=paste(b$names, " (n=", b$n, ")", sep=""))

par(mfrow=c(1, 1))
dev.off()


pdf("../results/likert_expertise.pdf", bg="transparent", width=12, height=4)
# Including missing values in table() results in R (http://stackoverflow.com/a/1617150)
s1_expertise_general_f <- table(factor(sample1$q10_expertise_general, levels=1:6)) 
s1_expertise_java_f <- table(factor(sample1$q11_expertise_java, levels=1:6))
s2_expertise_general_f <- table(factor(sample2$q3_expertise_general, levels=1:6)) 
s2_expertise_java_f <- table(factor(sample2$q4_expertise_java, levels=1:6)) 
s3_expertise_general_f <- table(factor(sample3$q3_expertise_general, levels=1:6)) 
s3_expertise_java_f <- table(factor(sample3$q4_expertise_java, levels=1:6)) 
tmp <- as.data.frame(rbind(s1_expertise_general_f,
                           s2_expertise_general_f,
                           s3_expertise_general_f,
                           s1_expertise_java_f,
                           s2_expertise_java_f,
                           s3_expertise_java_f))
tmp <- cbind(c("General S1", "General S2", "General S3", "Java S1", "Java S2", "Java S3"), tmp)
colnames(tmp) <- c("Variable", 1, 2, 3, 4, 5, 6)
HH::likert(Variable ~ ., data=tmp, main="Expertise Ratings", as.percent=FALSE, positive.order=FALSE, data.order=TRUE)
dev.off()


#### DEMOGRAPHICS MENTIONED IN THE TEXT ####

#### GENDER ####
table(gender)
# 1   2 
# 318   5 
table(sample1$q17_gender)
# 1   2 
# 115   1 
table(sample2$q17_gender)
# 1   2 
# 119   3
table(sample3$q17_gender)
# 1  2 
# 84  1


#### MAIN ROLE ####
main_role_t <- table(main_role)
main_role_t <- sort(main_role_t, decreasing=TRUE)
main_role_t[1]/length(main_role)
# 0.6328358  # software developers
main_role_t[2]/length(main_role)
# 0.1671642  # software architects

main_role_t_s1 <- table(sample1$q18_main_role)
main_role_t_s1 <- sort(main_role_t_s1, decreasing=TRUE)
main_role_t_s1[1]/length(sample1$q18_main_role)
# 0.6721311  # software developers
main_role_t_s1[2]/length(sample1$q18_main_role)
# 0.1393443  # software architects

main_role_t_s2 <- table(sample2$q18_main_role)
main_role_t_s2 <- sort(main_role_t_s2, decreasing=TRUE)
main_role_t_s2[1]/length(sample2$q18_main_role)
# 0.6062992  # software developers
main_role_t_s2[2]/length(sample2$q18_main_role)
# 0.1889764  # software architects

main_role_t_s3 <- table(sample3$q18_main_role)
main_role_t_s3 <- sort(main_role_t_s3, decreasing=TRUE)
main_role_t_s3[1]/length(sample3$q18_main_role)
# 0.6162791  # software developers
main_role_t_s3[2]/length(sample3$q18_main_role)
# 0.1744186  # software architects


#### CONTINENT ####
continent_t <- table(continent)
continent_t <- sort(continent_t, decreasing=TRUE)
continent_t[1]/length(na.omit(continent))
# 0.4328358  # North America
continent_t[2]/length(na.omit(continent))
# 0.4179104  # Europe

continent_t_s1 <- table(sample1$participant_continent)
continent_t_s1 <- sort(continent_t_s1, decreasing=TRUE)
continent_t_s1[1]/length(na.omit(sample1$participant_continent))
# 0.4918033   # North America
continent_t_s1[2]/length(na.omit(sample1$participant_continent))
# 0.3770492  # Europe

continent_t_s2 <- table(sample2$participant_continent)
continent_t_s2 <- sort(continent_t_s2, decreasing=TRUE)
continent_t_s2[1]/length(na.omit(sample2$participant_continent))
# 0.4724409  # North America
continent_t_s2[2]/length(na.omit(sample2$participant_continent))
# 0.3228346  # Europe

continent_t_s3 <- table(sample3$participant_continent)
continent_t_s3 <- sort(continent_t_s3, decreasing=TRUE)
continent_t_s3[1]/length(na.omit(sample3$participant_continent))
# 0.6744186  # North America
continent_t_s3[2]/length(na.omit(sample3$participant_continent))
# 0.2325581  # Europe

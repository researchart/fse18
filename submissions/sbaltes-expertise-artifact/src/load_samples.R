# set working directory (see https://stackoverflow.com/a/35842119)
dir = tryCatch({
  # script being sourced
  getSrcDirectory()[1]
}, error = function(e) {
  # script being run in RStudio
  dirname(rstudioapi::getActiveDocumentContext()$path)
})
setwd(dir)

# read result data
sample1 <- read.csv(file="../data/sample 1/sample1_export.csv", header=TRUE, sep=",", quote="\"", na.strings=c(""), stringsAsFactors=FALSE)
sample2 <- read.csv(file="../data/sample 2/sample2_export.csv", header=TRUE, sep=",", quote="\"", na.strings=c(""), stringsAsFactors=FALSE)
sample3 <- read.csv(file="../data/sample 3/sample3_export.csv", header=TRUE, sep=",", quote="\"", na.strings=c(""), stringsAsFactors=FALSE)

# recode variables
sample1$q20_degree_cs <- car::recode(sample1$q20_degree_cs, "5=NA;")  # recode "other" as NA
sample2$q21_degree_cs <- car::recode(sample2$q21_degree_cs, "5=NA;")  # recode "other" as NA
sample3$q21_degree_cs <- car::recode(sample3$q21_degree_cs, "5=NA;")  # recode "other" as NA
sample2$q15_dreyfus <- car::recode(sample2$q15_dreyfus, "6=NA;")  # recode "none of the stages" as NA
sample3$q15_dreyfus <- car::recode(sample3$q15_dreyfus, "6=NA;")  # recode "none of the stages" as NA
sample2$q10_monitoring <- car::recode(sample2$q10_monitoring, "3=NA;")  # recode "I don't want to answer" as NA
sample3$q10_monitoring <- car::recode(sample3$q10_monitoring, "3=NA;")  # recode "I don't want to answer" as NA
sample2$q12_decline <- car::recode(sample2$q12_decline, "3=NA;")  # recode "I don't want to answer" as NA
sample3$q12_decline <- car::recode(sample3$q12_decline, "3=NA;")  # recode "I don't want to answer" as NA
sample2$q13_mentor <- car::recode(sample2$q13_mentor, "3=NA;")  # recode "I don't want to answer" as NA
sample3$q13_mentor <- car::recode(sample3$q13_mentor, "3=NA;")  # recode "I don't want to answer" as NA

# merge samples
sample <- c(rep(1, nrow(sample1)), rep(2, nrow(sample2)), rep(3, nrow(sample3)))
gender <- c(sample1$q17_gender, sample2$q17_gender, sample3$q17_gender) 
experience_general <- c(sample1$q8_experience_general, sample2$q1_experience_general, sample3$q1_experience_general)
experience_java <- c(sample1$q9_experience_java, sample2$q2_experience_java, sample3$q2_experience_java)
expertise_general <- c(sample1$q10_expertise_general, sample2$q3_expertise_general, sample3$q3_expertise_general)
expertise_java <- c(sample1$q11_expertise_java, sample2$q4_expertise_java, sample3$q4_expertise_java)
age <- c(sample1$q16_age, sample2$q16_age, sample3$q16_age)
work_time <- c(sample1$q19_work_time, sample2$q19_work_time, sample3$q19_work_time)
degree_cs <- c(sample1$q20_degree_cs, sample2$q21_degree_cs, sample3$q21_degree_cs)
team_size <- c(rep(NA, nrow(sample1)), sample2$q20_team_size, sample3$q20_team_size)
dreyfus <- c(rep(NA, nrow(sample1)), sample2$q15_dreyfus, sample3$q15_dreyfus)
java_rating_time <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_time, sample3$q7_java_rating_time)
java_rating_prof_exp <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_prof_exp, sample3$q7_java_rating_prof_exp)
java_rating_depth <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_depth, sample3$q7_java_rating_depth)
java_rating_breadth <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_breadth, sample3$q7_java_rating_breadth)
java_rating_project_size <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_project_size, sample3$q7_java_rating_project_size)
java_rating_other_devs <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_other_devs, sample3$q7_java_rating_other_devs)
java_rating_trans_know <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_trans_know, sample3$q7_java_rating_trans_know)
java_rating_other <- c(rep(NA, nrow(sample1)), sample2$q7_java_rating_other, sample3$q7_java_rating_other)
monitoring <- c(rep(NA, nrow(sample1)), sample2$q10_monitoring, sample3$q10_monitoring)
decline <- c(rep(NA, nrow(sample1)), sample2$q12_decline, sample3$q12_decline)
mentor <- c(rep(NA, nrow(sample1)), sample2$q13_mentor, sample3$q13_mentor)
main_role <- c(sample1$q18_main_role, sample2$q18_main_role, sample3$q18_main_role)
continent <- c(sample1$participant_continent, sample2$participant_continent, sample3$participant_continent)

merged <- data.frame(sample, gender, experience_general, experience_java, expertise_general, expertise_java,
                     age, work_time, degree_cs, team_size, dreyfus, java_rating_time, java_rating_prof_exp,
                     java_rating_depth, java_rating_breadth, java_rating_project_size, java_rating_other_devs,
                     java_rating_trans_know, java_rating_other, monitoring, main_role, continent)
